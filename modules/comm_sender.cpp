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
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <ctime>
#include <iostream>

extern "C" {
    void run() {
        boost::mt19937 rng(std::time(0));
        boost::normal_distribution<> nd(0.0, 1.0);

        boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor(rng, nd);
        
                           
        CRAP::comm::send<std::string>("latchtest", "Latched string", true);
        while(true) {
            CRAP::comm::send<std::clock_t>("time", std::clock());
            CRAP::comm::send<double>("doubletest", var_nor());
            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        }
    }
}