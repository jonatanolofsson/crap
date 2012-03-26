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

#ifndef CRAP_FILTERING_HPP_
#define CRAP_FILTERING_HPP_

#include <Eigen/Core>
#include <iostream>

namespace CRAP {
    namespace filtering {
        using namespace Eigen;

        template<int number_of_observations, typename scalar = base_float_t>
        struct observation {
            typedef Matrix<scalar, number_of_observations, 1> measurement_t;
            typedef Matrix<scalar, number_of_observations, number_of_observations> covariance_t;
            measurement_t z;
            covariance_t R;
        };

        template<int number_of_states, typename scalar = base_float_t>
        struct prediction {
            typedef Matrix<scalar, number_of_states, 1> state_vector;
            typedef Matrix<scalar, number_of_states, number_of_states> state_covariance_matrix;

            state_vector x;
            state_covariance_matrix P;
        };

        template<int number_of_states, int number_of_controls, typename scalar = base_float_t>
        struct prediction_model_tmpl {
            typedef Matrix<scalar, number_of_states, 1> state_vector;
            typedef Matrix<scalar, number_of_controls, 1> control_vector;
            typedef Matrix<scalar, number_of_states, number_of_states> state_covariance_matrix;

            typedef const state_vector&(*prediction_fcn)(const state_vector&,const control_vector&);
            typedef const state_covariance_matrix&(*prediction_covariance_fcn)(const state_vector&, const control_vector&);

            prediction_fcn f;
            prediction_covariance_fcn Q;

            prediction_model_tmpl(prediction_fcn f_, prediction_covariance_fcn Q_) : f(f_), Q(Q_) {}
        };

        template<int number_of_states, int number_of_controls, typename scalar = base_float_t>
        struct KalmanFilter {
            typedef Matrix<scalar, number_of_states, 1> state_vector;
            typedef Matrix<scalar, number_of_states, number_of_states> state_covariance_matrix;

            state_vector x;
            state_covariance_matrix P;
        };
    }

    template<int number_of_states, int number_of_controls, typename scalar = base_float_t>
    std::ostream& operator<<(std::ostream& o, filtering::KalmanFilter<number_of_states, number_of_controls, scalar> kf) {
        return    o << ":::::::::: Kalman Filter ::::::::" << std::endl
                    << "State: " << std::endl << kf.x << std::endl
                    << "Covariance: " << std::endl << kf.P << std::endl
                    << ":::::::::::::::::::::::::::::::::" << std::endl;
    }
}

#endif
