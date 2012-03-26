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

#include "modules/controller/model.hpp"
#include "crap/communication.hpp"
#include <yaml-cpp/yaml.h>
#include <Eigen/Core>

namespace CRAP {
    namespace free_flight {
        using namespace Eigen;
        YAML::Node config;

        typedef bool(*has_ref_fn)(int&);
        typedef Vector4f(*ref_fn)();

        ref_fn get_joy = comm::bind<ref_fn>("baselink", "get_reference");
        has_ref_fn has_joy = comm::bind<has_ref_fn>("baselink", "has_new_reference");
        ::CRAP::controller::reference_vector ref;
        int last_joy;

        void send_reference() {
            if(has_joy(last_joy)) {
                ref = get_joy().cast<base_float_t>();
                //~ std::cout << "Sending reference: " << ref.transpose() << std::endl;

                CRAP::comm::send(config["reference_channel"].as<std::string>("/reference"), ref);
            }
        }
    }
}

extern "C" {
    void configure(YAML::Node& c) {CRAP::free_flight::config = c;}

    void* freeflight() {
        //~ std::cout << "Free flight" << std::endl;
        CRAP::free_flight::send_reference();
        return (void*) freeflight;
    }
}
