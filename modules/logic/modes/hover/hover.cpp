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

#include <iostream>
#include <Eigen/Core>
#include "modules/model/model.hpp"
#include "modules/controller/model.hpp"
#include "crap/communication.hpp"
#include <yaml-cpp/yaml.h>

extern "C" {
    YAML::Node config;
    void configure(YAML::Node& c) {config = c;}
    using namespace CRAP;
    using namespace Eigen;
    using namespace model;

    Matrix<base_float_t, 3, 1> position;
    state_vector x;
    ::CRAP::controller::reference_vector ref;

    typedef state_vector(*state_fn)();
    state_fn get_state = comm::bind<state_fn>("observer", "get_state");

    void* hover_control() {
        //~ std::cout << "Hovering control" << std::endl;
        //~ ref.setZero();
        x = get_state();
        {
            const base_float_t e = position.x() - x(state::positions[X]);
            static base_float_t pex = 0;
            ref.x() = config["control"]["x"]["P"].as<base_float_t>(1.0) * e + config["control"]["x"]["D"].as<base_float_t>(0.0) * (e - pex);
            pex = e;
        }
        {
            const base_float_t e = position.y() - x(state::positions[Y]);
            static base_float_t pey = 0;
            ref.y() = config["control"]["y"]["P"].as<base_float_t>(1.0) * e + config["control"]["y"]["D"].as<base_float_t>(0.0) * (e - pey);
            pey = e;
        }
        {
            const base_float_t e = position.z() - x(state::positions[Z]);
            static base_float_t pez = 0;
            ref.z() = config["control"]["z"]["P"].as<base_float_t>(1.0) * e + config["control"]["z"]["D"].as<base_float_t>(0.0) * (e - pez);
            pez = e;
        }
        CRAP::comm::send("/reference", ref);
        return (void*) hover_control;
    }

    void* zero_velocity() {
        ref.setZero();
        CRAP::comm::send("/reference", ref);
        return (NULL);
    }

    void* hover() {
        std::cout << "Hovering state 1" << std::endl;
        x = get_state();
        position = x.segment<3>(state::position);
        std::cout << "Hovering to position: " << position.transpose() << std::endl;
        //~ return (void*) zero_velocity;
        return (void*) hover_control;
    }
}
