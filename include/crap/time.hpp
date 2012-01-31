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

namespace CRAP {
    namespace time {
        struct frequency_t {
            double dt; ///< Seconds
            boost::timer t;
            bool first;

            frequency_t(double f) : dt(1.0/f), first(true) {}
            frequency_t() : first(true) {}
        };


        inline bool ticktock(frequency_t& t) {
            if(t.first) {
                t.first = false;
            }
            else {
                double sleep = t.dt - t.t.elapsed();
                if(sleep < 0.0) {
                    std::cerr << "Frequency of " << 1.0/double(t.dt) << " Hz not met by " << -sleep << " seconds" << std::endl;
                }
                else {
                    boost::this_thread::sleep(boost::posix_time::microseconds(std::floor(sleep*1e6)));
                }
            }
            t.t.restart();
            return true;
        }
    }
}
#endif
