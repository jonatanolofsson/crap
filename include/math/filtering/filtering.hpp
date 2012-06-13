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
#include <boost/thread/mutex.hpp>

namespace CRAP {
    namespace filtering {
        using namespace Eigen;

        template<int number_of_observations, typename scalar = base_float_t>
        struct observation {
            typedef Matrix<scalar, number_of_observations, 1> measurement_vector;
            typedef Matrix<scalar, number_of_observations, number_of_observations> covariance_matrix;
            measurement_vector z;
            covariance_matrix R;
            char flags;
            observation(){}
            observation(const measurement_vector& z_, const covariance_matrix& R_) : z(z_), R(R_){}
        };

        template<int number_of_states, int number_of_controls, typename scalar = base_float_t>
        struct prediction_model {
            typedef Matrix<scalar, number_of_states, 1> state_vector;
            typedef Matrix<scalar, number_of_controls, 1> control_vector;
            typedef Matrix<scalar, number_of_states, number_of_states> state_linearization_matrix;
            typedef Matrix<scalar, number_of_states, number_of_states> state_covariance_matrix;

            typedef const state_vector&(*prediction_fcn)(const state_vector&,const control_vector&);
            typedef const state_covariance_matrix&(*prediction_covariance_fcn)(const state_vector&, const control_vector&);



            prediction_fcn f;
            state_linearization_matrix F;
            state_linearization_matrix G;
            state_covariance_matrix Q;


            void linearize(const state_vector& x0,const control_vector& u0) {
                static const scalar delta = 1e-3;
                static const scalar div = 0.5/delta;

                // Change to forward (positive) difference only? Half the function eval's..
                state_vector x(x0);
                for(unsigned int i = 0; i < number_of_states; ++i) {
                    x[i] += delta;
                    F.col(i) = div*f(x,u0);
                    x[i] = x0[i]-delta;
                    F.col(i) -= div*f(x,u0);
                    x[i] = x0[i];
                }
            }
            const state_vector& operator()(const state_vector& x,const control_vector& u) { return f(x,u); }

            prediction_model(prediction_fcn f_, state_covariance_matrix Q_) : f(f_), Q(Q_) {G.setIdentity();}
            prediction_model(prediction_fcn f_) : f(f_) {G.setIdentity();}
        };

        template<int number_of_states, int number_of_controls, typename scalar = base_float_t>
        struct control_model {
            typedef Matrix<scalar, number_of_states, 1> state_vector;
            typedef Matrix<scalar, number_of_states, number_of_states> state_linearization_matrix;
            state_linearization_matrix F;
            state_vector x;
            control_model(const state_linearization_matrix& F_, const state_vector x_) : F(F_), x(x_) {}
        };

        template<int number_of_states, int number_of_observations, typename scalar = base_float_t>
        struct observation_model {
            typedef Matrix<scalar, number_of_states, 1> state_vector;
            typedef Matrix<scalar, number_of_observations, 1> measurement_vector;
            typedef Matrix<scalar, number_of_observations, number_of_states> linearization_matrix;

            typedef const measurement_vector&(*measurement_fcn)(const state_vector&);

            measurement_fcn h;
            linearization_matrix H;

            void linearize(const state_vector& x0) {
                static const scalar delta = 1e-3;
                static const scalar div = 0.5/delta;

                // Change to forward (positive) difference only? Half the function eval's..
                state_vector x(x0);
                for(unsigned int i = 0; i < number_of_states; ++i) {
                    x[i] += delta;
                    H.col(i) = div*h(x);
                    x[i] = x0[i]-delta;
                    H.col(i) -= div*h(x);
                    x[i] = x0[i];
                }
            }
            observation_model(measurement_fcn h_) : h(h_) {}
        };

        template<int number_of_states, int number_of_controls, typename scalar = base_float_t>
        struct KalmanFilter {
            boost::mutex lock;
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
