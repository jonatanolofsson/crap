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
#include "crap/modules/logic.hpp"
#include <iostream>

namespace CRAP {
    namespace logic {
        typedef std::map<std::string, void*> modemap_t;
        modemap_t modes;
        YAML::Node config;
        boost::mutex flightmode_lock;
        CRAP::time::frequency_t frequency;
        state_engine_t flightmode;

        bool load_mode(const std::string mode) {
            void* m;
            m = dlopen((config["modes"]["directory"].as<std::string>("") + mode + config["modes"]["extension"].as<std::string>(".so")).c_str(), RTLD_LAZY);
            if(m) {
                void* f;
                f = dlsym(m, mode.c_str());
                if(f) {
                    modes[mode] = f;
                    std::cout << "Loaded flightmode: " << mode << std::endl;
                    return true;
                }
                std::cout << "Function not found in loaded file: " << mode << std::endl;
            }
            std::cout << "Flightmode loading failed: " << mode << std::endl;
            return false;
        }

        bool on = true;

        void state_engine() {
            while(on && time::ticktock(frequency)) {
                flightmode_lock.lock();
                flightmode();
                flightmode_lock.unlock();
            }
        }

        void switch_mode(const std::string& mode) {
            boost::mutex::scoped_lock l(flightmode_lock);
            modemap_t::iterator it = modes.find(mode);
            if(it == modes.end()) {
                if(load_mode(mode)) {
                    flightmode.next(modes[mode]);
                }
            } else {
                flightmode.next(it->second);
            }
        }
    }
}

extern "C" {
    using namespace CRAP;
    void configure(YAML::Node& c) {
        logic::config = c;
        logic::frequency = time::frequency_t(c["frequency"].as<double>(50));
        logic::switch_mode(c["initial_mode"].as<std::string>());
    }

    void run() {
        comm::listen("/logic/mode", logic::switch_mode);
        logic::state_engine();
    }

    void shutdown() {
        logic::on = false;
    }
}
