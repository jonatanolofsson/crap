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

#ifndef CRAP_CORE_HPP_
#define CRAP_CORE_HPP_

#include "config.hpp"
#include <boost/timer.hpp>
namespace CRAP {
    extern boost::timer starting_time;

    typedef std::map<std::string, void*> module_map;

    /**
     * Map of all loaded modules
     */
    extern module_map modules;

    extern int argc;
    extern char** argv;
}

#endif
