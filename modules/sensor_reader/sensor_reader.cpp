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

namespace CRAP {
    namespace sensor_reader {
        using namespace LinkQuad::comm::serial::data;
        void smcu(const serial_data<SSMCU::gyro_data_0, SSMCU::gyro_data_1, SSMCU::gyro_data_2, SSMCU::accel_raw_0, SSMCU::accel_raw_1, SSMCU::accel_raw_2> d) {
            //~ std::cout << "Received gyro: " << d.gyro_data_0 << "\t" << d.gyro_data_1 << "\t" << d.gyro_data_2 << std::endl;
            std::cout << "Received acceleration: " << d.accel_raw_0 << "\t" << d.accel_raw_1 << "\t" << d.accel_raw_2 << std::endl;
        }

        YAML::Node config;
    }
}


extern "C" {
    using namespace CRAP::sensor_reader;
    void configure(YAML::Node& c) {
        config = c;
    }

    void run() {
        if(config["use_smcu"]) {
            LinkQuad::comm::serial::listen<SSMCU::Part>(config["smcu_port"], smcu);
        }
    }
}
