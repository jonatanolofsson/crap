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


#ifndef COMM_MESSAGES
#define COMM_MESSAGES

#include "linkquad/serial_communication.hpp"

#ifndef BIT_TST
    #define BIT_TST(data, bit_number) ((data) & (1 << (bit_number)))
    #define BIT_SET(data, bit_number) data |= (1 << (bit_number))
    #define BIT_CLR(data, bit_number) data &= ~(1 << (bit_number))
#endif

namespace CRAP {
    namespace comm_messages {
        using namespace LinkQuad::comm::serial::data;
        using namespace LinkQuad::comm::serial::data::SUser;

        enum comm_message_types {
            TRANSLATION_AND_ROTATION,
            BUTTON_PRESS,
            BUTTON_RELEASE
        };

        typedef serial_data<
            params_8i_0, // Flags
            params_32f_0, params_32f_1, params_32f_2, // Translation
            params_32f_3, // Rotation
            params_8i_1 // Button
        > command_message;

        typedef serial_data<
            params_32f_0, params_32f_1, params_32f_2, // Translation
            params_32f_3, params_32f_4, params_32f_5, params_32f_6 // Orientation
        > state_message;

        struct button_event {
            enum BUTTONS {LEFT = 0, RIGHT = 1};
            bool press;
            int button;
            button_event(bool p,int b) : press(p),button(b){}
        };
    }
}
#endif
