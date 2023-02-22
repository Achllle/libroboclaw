/**
 *
 * Copyright (c) 2018 Carroll Vance.
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

#ifndef PROJECT_ROBOCLAWDRIVER_H
#define PROJECT_ROBOCLAWDRIVER_H

#include <string>
#include <map>
#include <exception>

#include <boost/thread/mutex.hpp>
#include "TimeoutSerial.h"

namespace libroboclaw {

    template <typename T>
    using pair2 = std::pair<std::pair<T, T>, std::pair<T, T>>; 

    class driver {

    public:
        driver(std::string port, unsigned int baudrate);

        // Getters
        std::string get_version(unsigned char address);

        std::pair<int, int> get_encoders(unsigned char address);

        std::pair<int, int> get_velocity(unsigned char address);

        /**
         * Get the pwm values of the 2 motors at a given address
         */
        std::pair<int, int> get_pwm(const unsigned char address);
        
        /**
         * Get the percentage duty cycle of the 2 motors at a given address
         */
        std::pair<double, double> get_duty_cycle(const unsigned char address);

        /**
         * Get the current in centiamps (10mA increments) from the 2 motors at a given address
         e.g. a value of 3000 means 30 amps
        */
        std::pair<int, int> get_current(const unsigned char address);

        /**
         * Read errors if any are present
         * 
         * Normal 0x000000
         * E-Stop 0x000001
         * Temperature Error 0x000002
         * Temperature 2 Error 0x000004
         * Main Voltage High Error 0x000008
         * Logic Voltage High Error 0x000010
         * Logic Voltage Low Error 0x000020
         * M1 Driver Fault Error 0x000040
         * M2 Driver Fault Error 0x000080
         * M1 Speed Error 0x000100
         * M2 Speed Error 0x000200
         * M1 Position Error 0x000400
         * M2 Position Error 0x000800
         * M1 Current Error 0x001000
         * M2 Current Error 0x002000
         * M1 Over Current Warning 0x010000
         * M2 Over Current Warning 0x020000
         * Main Voltage High Warning 0x040000
         * Main Voltage Low Warning 0x080000
         * Temperature Warning 0x100000
         * Temperature 2 Warning 0x200000
         * S4 Signal Triggered 0x400000
         * S5 Signal Triggered 0x800000
         * Speed Error Limit Warning 0x01000000
         * Position Error Limit Warning 0x02000000        
         */
        int get_status(const unsigned char address);

        // config getters
        /**
         * Read Config
         * Options:
         *  RC Mode 0x0000
         *  Analog Mode 0x0001
         *  Simple Serial Mode 0x0002
         *  Packet Serial Mode 0x0003
         *  Battery Mode Off 0x0000
         *  Battery Mode Auto 0x0004
         *  Battery Mode 2 Cell 0x0008
         *  Battery Mode 3 Cell 0x000C
         *  Battery Mode 4 Cell 0x0010
         *  Battery Mode 5 Cell 0x0014
         *  Battery Mode 6 Cell 0x0018
         *  Battery Mode 7 Cell 0x001C
         *  Mixing 0x0020
         *  Exponential 0x0040
         *  MCU 0x0080
         *  BaudRate 2400 0x0000
         *  BaudRate 9600 0x0020
         *  BaudRate 19200 0x0040
         *  BaudRate 38400 0x0060
         *  BaudRate 57600 0x0080
         *  BaudRate 115200 0x00A0
         *  BaudRate 230400 0x00C0
         *  BaudRate 460800 0x00E0
         *  FlipSwitch 0x0100
         *  Packet Address 0x80 0x0000
         *  Packet Address 0x81 0x0100
         *  Packet Address 0x82 0x0200
         *  Packet Address 0x83 0x0300
         *  Packet Address 0x84 0x0400
         *  Packet Address 0x85 0x0500
         *  Packet Address 0x86 0x0600
         *  Packet Address 0x87 0x0700
         *  Slave Mode 0x0800
         *  Relay Mode 0X1000
         *  Swap Encoders 0x2000
         *  Swap Buttons 0x4000
         *  Multi-Unit Mode 0x8000
         *
         * So for example a value of 8355(dec) = 0010000010100011(bin) = 20A3 (hex)
         * would mean 0x2000 (swap encoders), 0x0000 (packet address 0x80/128),
         * 0x00A0 (baudrate 115200), 0x0003 (packet serial mode)        
         */
        int get_config(const unsigned char address);

        /**
         * Get the max current setting in centiamps from the 2 motors at a given address
         e.g. a value of 3000 means 30 amps
        */
        pair2<int> get_current_limits(const unsigned char address);

        // Setters
        /*
         * Drive M1 and M2 in the same command using a signed speed value. The sign indicates which
         * direction the motor will turn. This command is used to drive both motors by quad pulses per
         * second. Different quadrature encoders will have different rates at which they generate the
         * incoming pulses. The values used will differ from one encoder to another. Once a value is sent
         * the motor will begin to accelerate as fast as possible until the rate defined is reached.
         *
         * 4 Bytes (long) are used to express the pulses per second. Quadrature encoders send 4 pulses
         * per tick. So 1000 ticks would be counted as 4000 pulses. 
        */
        void set_velocity(unsigned char address, std::pair<int, int> speed);

        /*
         * Drive both M1 and M2 using a duty cycle value. The duty cycle is used to control the speed of
         * the motor without a quadrature encoder. 
         * The duty value is signed and the range is -32768 to +32767 (eg. +-100% duty). 
        */
        void set_duty(unsigned char address, std::pair<int, int> duty);

        /*
         * Set config bits for standard settings.
         * For possible values, see get_config
         * example:
         * To set the address of a roboclaw that's by default
         * at 128(dec) / 0x80(hex) to X (e.g. 129), call 
         * get_config(128) and add (X-128)*256 to the result,
         * then set_config() with this sum. The address should
         * have changed while other configuration will have
         * stayed the same.
        */
        void set_config(unsigned char address, int config);

        void reset_encoders(unsigned char address);

        static unsigned char BASE_ADDRESS;
        static unsigned int DEFAULT_BAUDRATE;

    private:
        std::shared_ptr<TimeoutSerial> serial;

        boost::asio::io_service io;

        boost::mutex serial_mutex;

        uint16_t crc;

        uint16_t crc16(uint8_t *packet, size_t nBytes);

        void crc16_reset();

        size_t txrx(unsigned char address, unsigned char command, unsigned char *tx_data, size_t tx_length,
                    unsigned char *rx_data, size_t rx_length, bool tx_crc = false, bool rx_crc = false);


    };

    class crc_exception : public std::runtime_error {
    public:
        using std::runtime_error::runtime_error;
    };

    // trim from start (in place)
    static inline void ltrim(std::string &s) {
        s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int ch) {
            return !std::isspace(ch);
        }));
    }

// trim from end (in place)
    static inline void rtrim(std::string &s) {
        s.erase(std::find_if(s.rbegin(), s.rend(), [](int ch) {
            return !std::isspace(ch);
        }).base(), s.end());
    }

// trim from both ends (in place)
    static inline void trim(std::string &s) {
        ltrim(s);
        rtrim(s);
    }
}
#endif //PROJECT_ROBOCLAWDRIVER_H