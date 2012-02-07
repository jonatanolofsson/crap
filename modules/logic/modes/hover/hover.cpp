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
#include "modules/observer/model.hpp"
#include "crap/communication.hpp"

extern "C" {
    using namespace CRAP;
    using namespace Eigen;
    using namespace observer::model;

    Matrix<double, 1,1> reference_position;

    typedef state_vector(*state_fn)();
    state_fn get_state = comm::bind<state_fn>("observer", "get_state");

    void* hover() {
        std::cout << "Hovering state 1" << std::endl;
        state_vector current_state = get_state();
        reference_position.x() = current_state(state::position);
        return (NULL);
    }
}
