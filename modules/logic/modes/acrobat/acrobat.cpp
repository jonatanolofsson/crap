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
#include <Eigen/Geometry>
#include <cmath>
#include "modules/model/model.hpp"
#include "modules/controller/model.hpp"
#include "modules/camera_reader/camera_data.hpp"
#include "crap/communication.hpp"
#include "linkquad/serial_communication.hpp"
#include "crap/base_types.hpp"
#include <yaml-cpp/yaml.h>

extern "C" {
    typedef CRAP::base_float_t scalar;
    YAML::Node config;
    void configure(YAML::Node& c) {config = c;}
    using namespace CRAP;
    using namespace Eigen;
    using namespace model;

    Matrix<base_float_t, 3, 1> center_position;
    state_vector x;
    ::CRAP::controller::reference_vector ref;

    typedef state_vector(*state_fn)();
    state_fn get_state = comm::bind<state_fn>("observer", "get_state");

    void* flight_control() {
        //~ std::cout << "Acrobat control" << std::endl;
        static double t = 0;
        Matrix<base_float_t, 3, 1> position = center_position + 0.5*AngleAxis<double>(t, Vector3d::UnitZ()).toRotationMatrix()*Vector3d::UnitX() + 0.25*std::sin(t)*Vector3d::UnitZ();
        ref.setZero();
        {
            const base_float_t e = position.x() - x(state::positions[X]);
            static base_float_t pe0 = 0;
            ref.x() = config["control"]["x"]["P"].as<base_float_t>(1.0) * e + config["control"]["x"]["D"].as<base_float_t>(0.0) * (e - pe0);
            pe0 = e;
        }
        {
            const base_float_t e = position.y() - x(state::positions[Y]);
            static base_float_t pe1 = 0;
            ref.y() = config["control"]["y"]["P"].as<base_float_t>(1.0) * e + config["control"]["y"]["D"].as<base_float_t>(0.0) * (e - pe1);
            pe1 = e;
        }
        {
            const base_float_t e = position.z() - x(state::positions[Z]);
            static base_float_t pe2 = 0;
            ref.z() = config["control"]["z"]["P"].as<base_float_t>(1.0) * e + config["control"]["z"]["D"].as<base_float_t>(0.0) * (e - pe2);
            pe2 = e;
        }
        ref(3) = 0.1;
        t += 1.0/300.0;
        if(t > 10) {
            CRAP::comm::send("/logic/mode", std::string("landing"));
        }
        CRAP::comm::send("/reference", ref);
        return (void*) flight_control;
    }
    CRAP::sensors::camera::serial_command cmd;
    void* acrobat() {
        std::cout << "Acrobat state 1" << std::endl;
        x = get_state();
        center_position = x.segment<3>(state::position) - Vector3d::UnitZ();
        return (void*) flight_control;
    }
}
