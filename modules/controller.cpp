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
#include "crap/modules/controller/model.hpp"
#include "crap/modules/observer/model.hpp"
#include "math/control/LQ.hpp"

namespace CRAP {
    namespace controller {
        control_vector u;
        reference_vector r;
        YAML::Node config;

        typedef control::LQ<model::number_of_states, model::number_of_controls> controller_type;
        controller_type reg;


        void calculate_control_signal(const observer::model::state_vector& state) {
            u = reg.control_signal(state, r);
            //~ cpplot::figure("Control signal") << u(0);
        }

        void init_controller(YAML::Node& c) {
            r.setZero(); u.setZero();
            config = c;
            control::linear_model<model::number_of_states, model::number_of_controls> lmodel;
            lmodel.A << 0, 1,
                        0, 0;
            lmodel.B << 0,
                        1;
            lmodel.M << 1,
                        0;
            lmodel.Q.setIdentity();
            lmodel.R.setIdentity();

            reg.set_model(lmodel);
        }
    }
}

// Module interface
extern "C" {
    using namespace CRAP::controller;

    control_vector control_signal() {
        return u;
    }
}

// CRAP interface
extern "C" {
    using namespace CRAP;
    void configure(YAML::Node& c) {
        controller::init_controller(c);
    }

    void run() {
        comm::listen("/state_estimate", controller::calculate_control_signal);
    }
}
