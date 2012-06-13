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

#ifndef CRAP_SENSOR_DATA_HPP_
#define CRAP_SENSOR_DATA_HPP_

#include "linkquad/serial_communication.hpp"

namespace CRAP {
    namespace sensors {
        namespace imu {
            using namespace LinkQuad::comm::serial;
            using namespace LinkQuad::comm::serial::data;
            using namespace LinkQuad::comm::serial::data::SSMCU;
            typedef serial_data<
                gyro_data_0, gyro_data_1, gyro_data_2,
                accel_raw_0, accel_raw_1, accel_raw_2,
                micromag_0, micromag_1, micromag_2,
                fAlt,
                system_status, battery_voltage
            > serial_data;
        }
    }
}

#endif
