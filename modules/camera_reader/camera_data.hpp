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

#ifndef CRAP_CAMERA_DATA_HPP_
#define CRAP_CAMERA_DATA_HPP_

#include <linkquad/serial_communication.hpp>

namespace CRAP {
    namespace sensors {
        namespace camera {
            using namespace LinkQuad::comm::serial;
            using namespace LinkQuad::comm::serial::data;
            using namespace LinkQuad::comm::serial::data::SUser;
            typedef LinkQuad::comm::serial::data::serial_data<
                params_32f_0,
                params_32f_1,
                params_32f_2,
                params_32f_3,
                params_32f_4,
                params_32f_5,
                params_32f_6,
                params_8i_0
            > serial_data;

            namespace quality {
                enum {BAD, DODGY, GOOD};
            }

            typedef LinkQuad::comm::serial::data::serial_data<params_8i_0> serial_command;

            namespace commands {
                enum {
                    STOP,
                    INITIALIZE
                };
            }
        }
    }
}

#endif
