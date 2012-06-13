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

    ::CRAP::controller::reference_vector ref;
    state_vector x;

    typedef state_vector(*state_fn)();
    state_fn get_state = comm::bind<state_fn>("observer", "get_state");

    void* spin_down() {
        std::cout << "Win!" << std::endl;
        return (NULL);
    }

    void* descend() {
        x = get_state();
        if(x(state::wind_velocities[Z]) < -config["landing_wind_threshold"].as<double>(5.0)) {
            std::cout << "Landing mode 3: Reached ground level." << std::endl;
            ref.setZero();
            CRAP::comm::send("/reference", ref);
            std::cout << " ::::::::   :::::::::::: Landing has been detected ::::::::: :::::::::::::::" << std::endl;
            return (void*) spin_down;
        }
        CRAP::comm::send("/reference", ref);
        return (void*) descend;
    }

    void* wait_for_halt() {
        x = get_state();
        if(x.segment<3>(state::velocity).norm() < config["halt_velocity"].as<state::scalar>(0.1)) {
            std::cout << "Landing mode 2: Halted, descending. " << std::endl;
            ref << 0,0,
                config["descend_speed"].as<state::scalar>(1.0),
                0;
            CRAP::comm::send("/reference", ref);
            return (void*) descend;
        }
        CRAP::comm::send("/reference", ref);
        return (void*) wait_for_halt;
    }

    void* landing() {
        std::cout << "Landing mode 1: Set reference to zero velocity and prepare descent." << std::endl;
        ref.setZero();
        CRAP::comm::send("/reference", ref);
        return (void*) wait_for_halt;
    }
}
