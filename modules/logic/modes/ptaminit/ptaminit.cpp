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
#include "modules/camera_reader/camera_data.hpp"
#include "crap/communication.hpp"
#include "linkquad/serial_communication.hpp"
#include "crap/base_types.hpp"
#include <yaml-cpp/yaml.h>
#include <unistd.h>

extern "C" {
    typedef CRAP::base_float_t scalar;
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

    void* flight_control() {
        //~ std::cout << "Hovering control" << std::endl;
        ref.setZero();
        {
            const base_float_t e = x(state::positions[X]) - position.x();
            static base_float_t pe = 0;
            ref.x() = config["control"]["x"]["P"].as<base_float_t>(1.0) * e + config["control"]["x"]["D"].as<base_float_t>(0.0) * (e - pe);
            pe = e;
        }
        {
            const base_float_t e = x(state::positions[Y]) - position.y();
            static base_float_t pe = 0;
            ref.y() = config["control"]["y"]["P"].as<base_float_t>(1.0) * e + config["control"]["y"]["D"].as<base_float_t>(0.0) * (e - pe);
            pe = e;
        }
        {
            const base_float_t e = x(state::positions[Z]) - position.z();
            static base_float_t pe = 0;
            ref.z() = config["control"]["z"]["P"].as<base_float_t>(1.0) * e + config["control"]["z"]["D"].as<base_float_t>(0.0) * (e - pe);
            pe = e;
        }
        CRAP::comm::send("/reference", ref);
        return (void*) flight_control;
    }
    CRAP::sensors::camera::serial_command cmd;
    void* ptaminit() {
        std::cout << "PTAM init state 1" << std::endl;
        sleep(4);
        x = get_state();
        Quaternion<scalar> qwb(
            x(state::quaternion_part_real),
            x(state::quaternion_part_vector[0]),
            x(state::quaternion_part_vector[1]),
            x(state::quaternion_part_vector[2])
        );
        position = x.segment<3>(state::position) - qwb.toRotationMatrix() * Matrix<scalar, 3, 1>::UnitY() * 0.3;

        cmd.params_8i_0 = CRAP::sensors::camera::commands::INITIALIZE;
        LinkQuad::comm::serial::send(config["port"].as<std::string>(), cmd);
        return (void*) flight_control;
    }
}
