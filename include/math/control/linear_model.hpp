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

#ifndef CRAP_MATH_CONTROL_LINEAR_MODEL_HPP_
#define CRAP_MATH_CONTROL_LINEAR_MODEL_HPP_

#include <Eigen/Core>
#include <Eigen/LU>

namespace CRAP {
    namespace control {
        using namespace Eigen;

        template<int number_of_states, int number_of_controls>
        struct linear_model {
            typedef Matrix<double, number_of_states, number_of_states> state_matrix;
            typedef Matrix<double, number_of_states, number_of_controls> control_matrix;
            typedef Matrix<double, number_of_controls, number_of_states> state_selection_matrix;
            typedef Matrix<double, number_of_controls, number_of_controls> state_weight_matrix;
            typedef Matrix<double, number_of_controls, number_of_controls> input_weight_matrix;

            char DICO; ///< 'D'iscrete or 'C'continous (default) model

            state_matrix A;
            control_matrix B;
            state_selection_matrix M;

            state_weight_matrix Q; ///< State weights for LQ-controller
            input_weight_matrix R; ///< Signal weights for LQ-controller
            linear_model() : DICO('C'){}
        };
    }
}

#endif
