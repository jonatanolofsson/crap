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

#include <map>

#include "serial_base.hpp"
#include <pthread.h>

namespace LinkQuad {
    namespace comm {
        namespace serial {
            typedef std::map<std::string, int> fdmap;
            fdmap serial_ports;
            pthread_mutex_t portmap_lock = PTHREAD_MUTEX_INITIALIZER;

            typedef std::map<std::string, serial_base*> listener_map;
            listener_map serial_listeners;

            typedef std::map<std::string, serial_talker*> talker_map;
            talker_map serial_talkers;

            typedef std::map<int, pthread_mutex_t*> mutexmap;
            mutexmap talker_mutexmap;
        }
    }
}
