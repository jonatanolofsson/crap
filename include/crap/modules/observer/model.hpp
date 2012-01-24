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

#ifndef CRAP_OBSERVER_MODEL_HPP_
#define CRAP_OBSERVER_MODEL_HPP_

#include <Eigen/Core>
#include "math/filtering/filtering.hpp"

namespace CRAP {
    namespace observer {
        using namespace Eigen;
        using namespace filtering;
        namespace model {
            namespace state {
                const int position = 0;
                const int velocity = 1;
            }

            namespace control {
                const int velocity = 0;
            }


            const double frequency                  = 30; ///< Hz
            const int number_of_states              = 2;
            const int number_of_controls            = 1;

            typedef Matrix<double, number_of_states, 1> state_vector;
            typedef Matrix<double, number_of_controls, 1> control_vector;
            typedef Matrix<double, number_of_states, number_of_states> state_covariance_matrix;
        }

        typedef KalmanFilter<model::number_of_states, model::number_of_controls> KF;
    }
}

#endif
