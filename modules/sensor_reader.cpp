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
        void smcu(const serial_data<SSMCU::accel_raw, SSMCU::gyro_data> d) {
            std::cout << "Received acceleration/gyro: " << d.accel_raw[0] << "\t" << d.accel_raw[1] << "\t" << d.accel_raw[2] << "\t" << d.gyro_data[0] << "\t" << d.gyro_data[1] << "\t" << d.gyro_data[2] << "\t" << std::endl;
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