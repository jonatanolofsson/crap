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

#ifndef CRAP_LINKQUAD_SERIAL_BASE
#define CRAP_LINKQUAD_SERIAL_BASE

#include "serial_data.hpp"

namespace LinkQuad {
    namespace comm {
        namespace serial {

            template<typename T>
            void* thread_initer(void* me) {
                ((T*)me)->init_thread();
                return (NULL);
            }

            class serial_base {
                public:
                    virtual ~serial_base() {}
            };
        }
    }
}
#endif
