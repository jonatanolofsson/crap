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
#include "boost/date_time/posix_time/posix_time.hpp"
#include <iostream>
#include <ctime>
#include <string>

void time_recipient(const std::clock_t& start) {
    std::clock_t now = std::clock();
    std::cout << (now - start)  << " (" <<  ( ( now - start ) / (double)CLOCKS_PER_SEC ) << ")\n";
}

void string_recipient(const std::string& msg) {
    std::cout << msg << std::endl;
}

extern "C" {
    void configure(YAML::Node& c) {
        std::cout << "Reconfiguring receiver" << std::endl;
    }
    void run() {
        CRAP::comm::listen<std::clock_t>("time", time_recipient);
        CRAP::comm::listen<std::string>("latchtest", string_recipient);
    }
}