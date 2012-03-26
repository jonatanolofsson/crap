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

#ifndef CRAP_TIME_HPP_
#define CRAP_TIME_HPP_

#include <boost/thread/thread.hpp>
#include <cmath>
#include "crap/base_types.hpp"

namespace CRAP {
    namespace time {
        typedef base_float_t dt_t;
        struct frequency_t {
            dt_t dt; ///< Seconds
            boost::timer t;
            bool first;

            frequency_t(dt_t f) : dt(1.0/f), first(true) {}
            frequency_t(dt_t t, bool) : dt(t), first(true) {}
            frequency_t() : first(true) {}
        };


        inline bool ticktock(frequency_t& t) {
            if(t.first) {
                t.first = false;
            }
            else {
                dt_t sleep = t.dt - t.t.elapsed();
                if(sleep > 0.0) {
                    boost::this_thread::sleep(boost::posix_time::microseconds(std::floor(sleep*1e6)));
                }
                //~ else {
                    //~ dt_t freq = 1.0/t.dt;
                    //~ std::cerr << "Frequency of " << freq << " Hz not met by " << -sleep << " seconds (" << -sleep*freq*100.0 << " %)" << std::endl;
                //~ }
            }
            t.t.restart();
            return true;
        }
    }
}
#endif
