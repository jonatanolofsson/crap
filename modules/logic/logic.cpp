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
#include "crap/state_engine.hpp"
#include "modules/logic/governor.hpp"
#include <iostream>

namespace CRAP {
    namespace logic {
        CRAP::time::frequency_t frequency;

        bool on = true;

        void state_engine() {
            while(on && time::ticktock(frequency)) {
                governor::tick();
            }
        }
    }
}

extern "C" {
    using namespace CRAP;
    void configure(YAML::Node& c) {
        logic::governor::configure(c);
        logic::frequency = time::frequency_t(c["frequency"].as<base_float_t>(30));
        logic::governor::switch_mode(c["initial_mode"].as<std::string>());
    }

    void run() {
        comm::listen("/logic/mode", logic::governor::switch_mode);
        logic::state_engine();
    }

    void shutdown() {
        logic::on = false;
    }
}
