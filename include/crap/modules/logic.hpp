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

#ifndef CRAP_LOGIC_HPP_
#define CRAP_LOGIC_HPP_
namespace CRAP {
    namespace logic {
        class state_engine_t {
            private:
                void* next_state;
            public:
                typedef void*(*next_t)();

                state_engine_t(void* n) : next_state(n) {}
                state_engine_t() {}

                void operator()() { next_state = ((next_t)next_state)(); }
                void next(next_t n) { next_state = (void*)n; }
                void next(void* n) { next_state = n; }
        };
    }
}
#endif
