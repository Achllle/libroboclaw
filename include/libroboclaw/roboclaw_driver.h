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
         TODO: declare possibilities, use enum?
        */
        int get_error(const unsigned char address);

        // config getters
        /**
         * Get the max current setting in centiamps from the 2 motors at a given address
         e.g. a value of 3000 means 30 amps
        */
        std::pair<int, int> get_max_current(const unsigned char address);

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
         * The duty value is signed and the range is +-1500. 
        */
        void set_duty(unsigned char address, std::pair<int, int> duty);

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