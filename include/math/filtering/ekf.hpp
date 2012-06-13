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

#ifndef CRAP_FILTERING_UKF_HPP_
#define CRAP_FILTERING_UKF_HPP_

#include "filtering.hpp"
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <cmath>

#include <iostream>

namespace CRAP {
    namespace filtering {
        using namespace Eigen;
        namespace ekf {

            /**
             */
            template<int number_of_states, int number_of_controls, typename scalar = base_float_t>
            void predict(
                KalmanFilter<number_of_states, number_of_controls, scalar>& filter,
                prediction_model<number_of_states, number_of_controls, scalar>& f,
                const Matrix<scalar, number_of_controls, 1>& u)
            {
                f.linearize(filter.x,u);
                //~ std::cout << " Prediction! ::::::::::::::::: u: " << u.transpose() << "::::::::::::::::" << std::endl;
                //~ std::cout << "x before: " << filter.x.transpose() << std::endl;
                //~ std::cout << "F:" << std::endl << f.F << std::endl << std::endl;
                filter.x = f.f(filter.x, u);
                filter.P = f.F * filter.P * f.F.transpose() +  f.G * f.Q * f.G.transpose();
                //~ std::cout << "x after:  " << filter.x.transpose() << std::endl;
            }


            /**
             */
            template<int number_of_states, int number_of_controls, int number_of_observations, typename scalar = base_float_t>
            void observe(
                KalmanFilter<number_of_states, number_of_controls, scalar>& filter,
                observation_model<number_of_states, number_of_observations, scalar>& h,
                const observation<number_of_observations, scalar>& o)
            {
                //~ std::cout << " Measurement! (" << number_of_observations << ") ::::::::::::::::: Min P: " << filter.P.minCoeff() << "::::: Max P: " << filter.P.maxCoeff() << " :::::::::::::::" << std::endl;
                h.linearize(filter.x);
                //~ std::cout << "H: \n" << h.H << std::endl;
                //~ std::cout << "R: \n" << o.R << std::endl;
                //~ std::cout << "P before: \n" << filter.P << std::endl;
                const Matrix<scalar, number_of_states, number_of_observations> KA = filter.P * h.H.transpose();
                const LDLT<Matrix<scalar, number_of_observations, number_of_observations> > KI = (h.H * filter.P * h.H.transpose() + o.R).ldlt();
                //~ std::cout << "Innovation: \n" << o.z.transpose() << "\n\t - \n" << h.h(filter.x).transpose() << "\n\t = \n" << (o.z - h.h(filter.x)).transpose() << std::endl;
                //~ std::cout << "Before: " << filter.x(state::camera_scale) << std::endl;
                //~ std::cout << "Before: " << filter.x.transpose() << std::endl;
                //~ std::cout << filter.x.transpose() << std::endl;
                //~ std::cout << "Delta:  " << (KA * KI.solve(o.z - h.h(filter.x))).transpose() << std::endl;
                filter.x += KA * KI.solve(o.z - h.h(filter.x));
                filter.P -= KA * KI.solve(h.H) * filter.P;
                //~ std::cout << "After: " << filter.x(state::camera_scale) << std::endl;
                //~ std::cout << "After:  " << filter.x.transpose() << std::endl << std::endl;
                //~ std::cout << "P after: \n" << filter.P << std::endl;
                //~ std::cout << "::::::::::::::::: Min P: " << filter.P.minCoeff() << "::::: Max P: " << filter.P.maxCoeff() << " :::::::::::::::" << std::endl;
            }
        }
    }
}

#endif
