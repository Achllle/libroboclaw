/**
 *
 * Copyright (c) 2018 Carroll Vance, Achille Verheye
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */
#include <iostream>
#include "libroboclaw/roboclaw_driver.h"

#include <boost/thread/mutex.hpp>

namespace libroboclaw {

    unsigned char driver::BASE_ADDRESS = 128;
    unsigned int driver::DEFAULT_BAUDRATE = 115200;

    driver::driver(std::string port, unsigned int baudrate) {
        serial = std::shared_ptr<TimeoutSerial>(new TimeoutSerial(port, baudrate));
        serial->setTimeout(boost::posix_time::milliseconds(200));
    }

    void driver::crc16_reset() {
        crc = 0;
    }

    uint16_t driver::crc16(uint8_t *packet, size_t nBytes) {

        for (size_t byte = 0; byte < nBytes; byte++) {

            crc = crc ^ ((uint16_t) packet[byte] << 8);

            for (uint8_t bit = 0; bit < 8; bit++) {
                if (crc & 0x8000)
                    crc = (crc << 1) ^ 0x1021;
                else
                    crc = crc << 1;
            }
        }

        return crc;
    }

    size_t driver::txrx(unsigned char address,
                        unsigned char command,
                        unsigned char *tx_data,
                        size_t tx_length,
                        unsigned char *rx_data,
                        size_t rx_length,
                        bool tx_crc, bool rx_crc) {
        /*
         * All packet serial commands use a 7 bit checksum to prevent corrupt commands from being
         * executed. Since the RoboClaw expects a 7bit value the 8th bit is masked. The checksum is
         * calculated as follows:
         * Checksum = (Address + Command + Data bytes sent + Data bytes read ) & 0x7F
         * When calculating the checksum all data bytes sent or received must be added together. The
         * hexadecimal value 0X7F is used to mask the 8th bit.
        */
        // usleep(1000);
        // ioctl(0x80->fd, TCFLSH, 0); // flush receive
        // ioctl(0x80, TCFLSH, 1); // flush transmit
        // ioctl(0x80, TCFLSH, 2); // flush both
        usleep(15000);

        boost::mutex::scoped_lock lock(serial_mutex);

        std::vector<unsigned char> packet;

        // minimum packet size is 2 (address byte and command byte). Add two bytes for 16bit crc
        // when using transmission checksum
        if (tx_crc)
            packet.resize(tx_length + 4);
        else
            packet.resize(tx_length + 2);

        // Header
        packet[0] = address;
        packet[1] = command;

        crc16_reset();
        crc16(&packet[0], 2);

        // Data
        if (tx_length > 0 && tx_data != nullptr)
            memcpy(&packet[2], tx_data, tx_length);

        // CRC
        if (tx_crc) {
            unsigned int crc = crc16(&packet[2], tx_length);

            // RoboClaw expects big endian / MSB first
            packet[tx_length + 2] = (unsigned char) ((crc >> 8) & 0xFF);
            packet[tx_length + 2 + 1] = (unsigned char) (crc & 0xFF);

        }

        serial->write((char*)&packet[0], packet.size());

        size_t want_bytes;
        if (rx_crc)
            want_bytes = rx_length + 2;
        else
            want_bytes = rx_length;

        std::vector<char> response_vector;

        response_vector = serial->read(want_bytes);

        size_t bytes_received = response_vector.size();
        // for (char i:response_vector)
        //     std::cout << (int) i << ' ';
        // std::cout << std::endl;

        unsigned char* response = (unsigned char*) &response_vector[0];

        if (bytes_received != want_bytes) {
            std::cout << "error: " << bytes_received << " received vs " << want_bytes << " expected." << std::endl;
            throw timeout_exception("Timeout reading from RoboClaw or # bytes mismatch");
        }

        // Check CRC
        if (rx_crc) {
            uint16_t crc_calculated = crc16(&response[0], bytes_received - 2);
            uint16_t crc_received = 0;

            // RoboClaw generates big endian / MSB first
            crc_received += response[bytes_received - 2] << 8;
            crc_received += response[bytes_received - 1];

            if (crc_calculated != crc_received) {
                std::cout << "error: crc_calculated: " << crc_calculated << ", crc_received: " << crc_received << std::endl;
                throw crc_exception("Roboclaw CRC mismatch");
            }

            memcpy(rx_data, &response[0], bytes_received - 2);
        } else {
            memcpy(rx_data, &response[0], bytes_received);
        }

        if (!rx_crc)
            return bytes_received;
        else
            return bytes_received - 2;
    }

    std::string driver::get_version(unsigned char address) {
        /*
         * The command will return up to 32 bytes. The return string includes the product name and
         * firmware version. The return string is terminated with a null (0) character.
        */
        // unsigned char rx_buffer[30];
        // txrx(address, 21, nullptr, 0, rx_buffer, 30, false, false);
        std::string version;
        for (int i=0; i<32; i++){
            unsigned char rx_buffer[i];
            try {
            txrx(address, 21, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, false);
            version = std::string(reinterpret_cast< char const * >(rx_buffer));
            trim(version);
            } catch (...) {}
        }
        return version;
    }

    std::pair<int, int> driver::get_encoders(unsigned char address) {
        /*
         * Read decoder M1 counter. Since CMD 16 is a read command it does not require a checksum.
         * However a checksum value will be returned from RoboClaw and can be used to validate the data

         * Send: [Address, CMD]
         * Receive: [Value1.Byte3, Value1.Byte2, Value1.Byte1, Value1.Byte0, Value2, Checksum]

         * The command will return 6 bytes. Byte 1,2,3 and 4 make up a long variable which is received
         * MSB first and represents the current count which can be any value from 0 - 4,294,967,295. Each
         * pulse from the quadrature encoder will increment or decrement the counter depending on the
         * direction of rotation.

         * Byte 5 is the status byte for M1 decoder. It tracks counter underflow, direction, overflow and if
         * the encoder is operational. The byte value represents:
         *      Bit0 - Counter Underflow (1= Underflow Occurred, Clear After Reading)
         *      Bit1 - Direction (0 = Forward, 1 = Backwards)
         *      Bit2 - Counter Overflow (1= Underflow Occurred, Clear After Reading)
         *      Bit3 - Reserved
         *      Bit4 - Reserved
         *      Bit5 - Reserved
         *      Bit6 - Reserved
         *      Bit7 - Reserved
         * Byte 6 is the checksum. It is calculated the same way as sending a command, Sum all the
         * values sent and received except the checksum and mask the 8th bit.
         * 
         * Repeat for M2
        */

        unsigned char rx_buffer[5];

        txrx(address, 16, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

        uint32_t e1 = 0;

        e1 += rx_buffer[0] << 24;
        e1 += rx_buffer[1] << 16;
        e1 += rx_buffer[2] << 8;
        e1 += rx_buffer[3];

        txrx(address, 17, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

        uint32_t e2 = 0;

        e2 += rx_buffer[0] << 24;
        e2 += rx_buffer[1] << 16;
        e2 += rx_buffer[2] << 8;
        e2 += rx_buffer[3];

        return std::pair<int, int>((int) (int32_t) e1, (int) (int32_t) e2);
    }

    std::pair<int, int> driver::get_velocity(unsigned char address) {

        unsigned char rx_buffer[5];

        txrx(address, 18, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

        uint32_t e1 = 0;

        e1 += rx_buffer[0] << 24;
        e1 += rx_buffer[1] << 16;
        e1 += rx_buffer[2] << 8;
        e1 += rx_buffer[3];

        txrx(address, 19, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

        uint32_t e2 = 0;

        e2 += rx_buffer[0] << 24;
        e2 += rx_buffer[1] << 16;
        e2 += rx_buffer[2] << 8;
        e2 += rx_buffer[3];

        return std::pair<int, int>((int) (int32_t) e1, (int) (int32_t) e2);
    }

    std::pair<int, int> driver::get_pwm(const unsigned char address){
        unsigned char rx_buffer[4];
        uint16_t pwm_1 = 0;
        uint16_t pwm_2 = 0;
        
        txrx(address, 48, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

        pwm_1 += rx_buffer[0] << 8;
        pwm_1 += rx_buffer[1];
        pwm_2 += rx_buffer[2] << 8;
        pwm_2 += rx_buffer[3];

        return std::pair<int, int>((int) (int16_t) pwm_1, (int) (int16_t) pwm_2);
    }

    std::pair<double, double> driver::get_duty_cycle(const unsigned char address){
        std::pair<int, int> pwm;
        std::pair<double, double> duty_cycle;
        double max_pwm = 32767;  // Maximum absolute pwm value. Refer to roboclaw user manual for more information. 

        pwm = get_pwm(address);
        duty_cycle.first = 100.0 * (double) pwm.first/max_pwm;
        duty_cycle.second = 100.0 * (double) pwm.second/max_pwm;

        return duty_cycle;
    }

    std::pair<int, int> driver::get_current(const unsigned char address) {
        /*
         * Send: [Address, 49]
         * Receive: [M1Cur.Byte1, M1Cur.Byte0, M2Cur.Byte1, M2Cur.Byte0, Checksum]

         * The command will return 5 bytes. Bytes 1 and 2 combine to represent the current in 10ma
         * increments of motor1. Bytes 3 and 4 combine to represent the current in 10ma increments of
         * motor2 . Byte 5 is the checksum.
        */
        unsigned char rx_buffer[4];
        uint16_t curr_1 = 0;
        uint16_t curr_2 = 0;

        txrx(address, 49, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

        curr_1 += rx_buffer[0] << 8;
        curr_1 += rx_buffer[1];
        curr_2 += rx_buffer[2] << 8;
        curr_2 += rx_buffer[3];

        return std::pair<int, int>((int) (int16_t) curr_1, (int) (int16_t) curr_2);
    }

    int driver::get_status(const unsigned char address) {
        /*
         * Send: [Address, 90]
         * Receive: [Status (4 bytes), CRC(2 bytes)]
        */
        unsigned char rx_buffer[4];
        uint32_t error = 0;
        
        txrx(address, 90, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

        error += rx_buffer[0] << 24;
        error += rx_buffer[1] << 16;
        error += rx_buffer[2] << 8;
        error += rx_buffer[3];

        return (int) error;
    }

    int driver::get_config(const unsigned char address) {
        /*
         * Send: [Address, 99]
         * Receive: [Config (2 bytes), CRC(2 bytes)]
        */
        unsigned char rx_buffer[2];
        uint16_t config = 0;
        
        txrx(address, 99, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

        config += rx_buffer[0] << 8;
        config += rx_buffer[1];

        return (int) config;
    }

    pair2<int> driver::get_current_limits(const unsigned char address) {
        /*
         * Send: [Address, 135 (M1) / 136 (M2)]
         * Receive: [MaxCurrent(4 bytes), MinCurrent(4 bytes), CRC(2 bytes)]
        */
        unsigned char rx_buffer[8];

        txrx(address, 135, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

        uint32_t max_curr_1 = 0;
        max_curr_1 += rx_buffer[0] << 24;
        max_curr_1 += rx_buffer[1] << 16;
        max_curr_1 += rx_buffer[2] << 8;
        max_curr_1 += rx_buffer[3];

        uint32_t min_curr_1 = 0;
        min_curr_1 += rx_buffer[4] << 24;
        min_curr_1 += rx_buffer[5] << 16;
        min_curr_1 += rx_buffer[6] << 8;
        min_curr_1 += rx_buffer[7];

        std::pair<int, int> max_min1;
        max_min1 = std::pair<int, int>((int) (int32_t) max_curr_1, (int) (int32_t) min_curr_1);

        txrx(address, 136, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

        uint32_t max_curr_2 = 0;
        max_curr_2 += rx_buffer[0] << 24;
        max_curr_2 += rx_buffer[1] << 16;
        max_curr_2 += rx_buffer[2] << 8;
        max_curr_2 += rx_buffer[3];

        uint32_t min_curr_2 = 0;
        min_curr_2 += rx_buffer[4] << 24;
        min_curr_2 += rx_buffer[5] << 16;
        min_curr_2 += rx_buffer[6] << 8;
        min_curr_2 += rx_buffer[7];

        std::pair<int, int> max_min2;
        max_min2 = std::pair<int, int>((int) (int32_t) max_curr_2, (int) (int32_t) min_curr_2);

        return pair2<int>(max_min1, max_min2);
    }

    void driver::set_velocity(unsigned char address, std::pair<int, int> speed) {
        /* 
         * Send: [Address, CMD, QspeedM1(4 Bytes), QspeedM2(4 Bytes), Checksum] 
         * Receive: [0xFF]
        */
        unsigned char rx_buffer[1];
        unsigned char tx_buffer[8];

        // RoboClaw expects big endian / MSB first
        tx_buffer[0] = (unsigned char) ((speed.first >> 24) & 0xFF);
        tx_buffer[1] = (unsigned char) ((speed.first >> 16) & 0xFF);
        tx_buffer[2] = (unsigned char) ((speed.first >> 8) & 0xFF);
        tx_buffer[3] = (unsigned char) (speed.first & 0xFF);

        tx_buffer[4] = (unsigned char) ((speed.second >> 24) & 0xFF);
        tx_buffer[5] = (unsigned char) ((speed.second >> 16) & 0xFF);
        tx_buffer[6] = (unsigned char) ((speed.second >> 8) & 0xFF);
        tx_buffer[7] = (unsigned char) (speed.second & 0xFF);

        txrx(address, 37, tx_buffer, sizeof(tx_buffer), rx_buffer, sizeof(rx_buffer), true, false);
    }

    void driver::set_duty(unsigned char address, std::pair<int, int> duty) {
        /*
         * Send: [Address, CMD, DutyM1(2 Bytes), DutyM2(2 Bytes), CRC(2 bytes)]
         * Receive: [0xFF]
        */
        unsigned char rx_buffer[1];
        unsigned char tx_buffer[4];

        // RoboClaw expects big endian / MSB first
        tx_buffer[0] = (unsigned char) ((duty.first >> 8) & 0xFF);
        tx_buffer[1] = (unsigned char) (duty.first & 0xFF);

        tx_buffer[2] = (unsigned char) ((duty.second >> 8) & 0xFF);
        tx_buffer[3] = (unsigned char) (duty.second & 0xFF);

        txrx(address, 34, tx_buffer, sizeof(tx_buffer), rx_buffer, sizeof(rx_buffer), true, false);
    }

    void driver::reset_encoders(unsigned char address) {
        /*
         * Send: [Address, CMD]
         * Receive: [0xFF]
        */
        unsigned char rx_buffer[1];
        txrx(address, 20, nullptr, 0, rx_buffer, sizeof(rx_buffer), true, false);
    }

    void driver::set_config(unsigned char address, int config) {
        /*
         * Send: [Address, 98, Config(2 bytes), CRC(2 bytes)]
         * Receive: [0xFF]
        */
        unsigned char rx_buffer[1];
        unsigned char tx_buffer[2];

        // RoboClaw expects big endian / MSB first
        tx_buffer[0] = (unsigned char) ((config >> 8) & 0xFF);
        tx_buffer[1] = (unsigned char) (config & 0xFF);

        txrx(address, 98, tx_buffer, sizeof(tx_buffer), rx_buffer, sizeof(rx_buffer), true, false);
    }

}