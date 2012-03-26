/**
 * Copyright 2011, 2012 Jonatan Olofsson
 *
 * This file is part of C++ Robot Automation Platform (CRAP).
 *
 * CRAP is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CRAP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with CRAP.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "crap/module.hpp"
#include "linkquad/serial_communication.hpp"

#include <iostream>
#include <iomanip>

namespace CRAP {
    namespace sensor_reader {
        YAML::Node config;
        base_float_t scales[3];
        namespace sensors {
            static const int accelerometer  = 0;
            static const int gyro           = 1;
            static const int battery        = 2;
        }

        using namespace LinkQuad::comm::serial::data;
        using namespace LinkQuad::comm::serial::data::SSMCU;
        void smcu(const serial_data<
            gyro_data_0, gyro_data_1, gyro_data_2,
            accel_raw_0, accel_raw_1, accel_raw_2,
            micromag_0, micromag_1, micromag_2,
            fAlt,
            system_status, battery_voltage
        > d) {
            std::cout
                << std::setw(12) << (float)d.fAlt
                << std::setw(12) << (int)d.system_status
                << std::setw(12) << (int)d.battery_voltage * scales[sensors::battery]

                << std::setw(12) << (float)d.accel_raw_0 * scales[sensors::accelerometer]
                << std::setw(12) << (float)d.accel_raw_1 * scales[sensors::accelerometer]
                << std::setw(12) << (float)d.accel_raw_2 * scales[sensors::accelerometer]

                << std::setw(12) << (int)d.gyro_data_0 * scales[sensors::gyro]
                << std::setw(12) << (int)d.gyro_data_1 * scales[sensors::gyro]
                << std::setw(12) << (int)d.gyro_data_2 * scales[sensors::gyro]

                << std::setw(12) << (int)d.micromag_0
                << std::setw(12) << (int)d.micromag_1
                << std::setw(12) << (int)d.micromag_2

                << std::endl;
        }

    }
}


extern "C" {
    using namespace CRAP;
    using namespace CRAP::sensor_reader;
    using namespace LinkQuad::comm::serial::data;
    void configure(YAML::Node& c) {
        config = c;
        scales[sensors::accelerometer] = c["scales"]["accelerometer"].as<base_float_t>();
        scales[sensors::gyro] = c["scales"]["gyro"].as<base_float_t>();
        scales[sensors::battery] = c["scales"]["gyro"].as<base_float_t>();
    }

    void run() {
        if(config["use_smcu"]) {
            LinkQuad::comm::serial::listen<SSMCU::Part>(config["smcu_port"].as<std::string>(), smcu);
        }
    }
}
