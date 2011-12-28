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
#include <string>
#include "boost/date_time/posix_time/posix_time.hpp"
#include <ctime>

extern "C" {
    void run() {
        CRAP::comm::send<std::string>("latchtest", "Latched string", true);
        while(true) {
            CRAP::comm::send<std::clock_t>("time", std::clock());
            boost::this_thread::sleep(boost::posix_time::milliseconds(100));
        }
    }
}