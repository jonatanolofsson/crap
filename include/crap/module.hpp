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

#ifndef CRAP_MODULE_HPP_
#define CRAP_MODULE_HPP_

#include "crap/config.hpp"
#include "crap/communication.hpp"
#include <boost/thread.hpp>
#include "yaml-cpp/yaml.h"
#include "crap/log.hpp"
#include "crap/core.hpp"
#include "crap/time.hpp"
#include "crap/base_types.hpp"

#ifdef CRAP_PLOT
    #include "cpplot/cpplot.hpp"
#endif

#ifdef CRAP_OUTPUT_DATA
    #include <fstream>
#endif

//CRAP_MODULE_HPP_
#endif
