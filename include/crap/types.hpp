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

#ifndef CRAP_TYPES_HPP_
#define CRAP_TYPES_HPP_

#include <list>
#include <map>
#include <string>

namespace CRAP {
    namespace comm {
        typedef const void* listener_t;
        typedef std::list<listener_t> listeners_t;
        typedef std::map<std::string, listeners_t> listener_map_t;

        typedef std::map<std::string, void*> latched_messages_map_t;

        struct messagequeue_base_t {virtual ~messagequeue_base_t(){}};
        typedef std::map<std::string, messagequeue_base_t*> messagequeues_t;
    }
}

// CRAP_TYPES_HPP_
#endif
