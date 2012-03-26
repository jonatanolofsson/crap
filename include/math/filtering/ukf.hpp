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
        namespace ukf {
            namespace detail {
                /**
                 * Create a set of sigma points for the unscented transform
                 * @param XX [Output] Matrix where to put each sigma point as a column
                 * @param x Center point of transform
                 * @param P Covariance used in transform to distribute the sigma points
                 * @param scale Scaling constant of transform
                 * @return void
                 */
                template<int number_of_states, typename scalar = base_float_t>
                void unscented(
                    Matrix<scalar, number_of_states, 2*number_of_states+1>& XX,
                    const Matrix<scalar, number_of_states, 1>& x,
                    const Matrix<scalar, number_of_states, number_of_states>& P,
                    const scalar scale)
                /*
                 * Generate the unscented point representing a distribution
                 * Fails if scale is negative
                 */
                {
                    typedef Matrix<scalar, number_of_states, number_of_states> state_covariance_matrix;
                    // Get a Cholesky factoriation
                    // The original implementation says that it uses the
                    // upper triangular form, but this is not consistent
                    // with the formulas they say they're using.
                    // Also, http://en.wikipedia.org/wiki/Cholesky_decomposition#Kalman_filters
                    // suggests that it is in fact the lower triangular form that
                    // should be used. //Jonatan Olofsson
                    state_covariance_matrix sigma = (state_covariance_matrix)(P.llt().matrixL())*scale;

                    // Generate XX with the same sample Mean and Covar as before
                    XX.col(0) = x;

                    for (std::size_t c = 0; c < number_of_states; ++c) {
                        XX.col(c+1) = x + sigma.col(c);
                        XX.col(number_of_states + c+1) = x - sigma.col(c);
                    }
                    //~ std::cout << "\n :: Sigma points ::< \n" << XX << "\n> ::::::::::::::::::::::" << std::endl;
                }
            }

            /**
             * Perform a prediction update by generating a set of sigma points
             * and propagating them all trough the model and interpret the result
             * @param f Prediction model containing the model function and prediction covariance
             * @param u Control signal
             * @return void. The internal state of the filter is modified.
             */
            template<int number_of_states, int number_of_controls, int starting_state = 0, typename scalar = base_float_t>
            void predict(
                KalmanFilter<number_of_states, number_of_controls, scalar>& filter,
                const prediction_model_tmpl<number_of_states, number_of_controls, scalar>& f,
                const Matrix<scalar, number_of_controls, 1>& u,
                const scalar alpha = 1e-2, const scalar beta = 2.0, const scalar kappa = 0)
            {
                Matrix<scalar, number_of_states, 2*number_of_states+1> XX, fXX;
                Matrix<scalar, number_of_states, number_of_states> P;
                Matrix<scalar, number_of_states, 1> x;
                //~ std::cout << "Pre-time update::::" << std::endl;
                //~ std::cout << "x:\n" << x << std::endl;
                //~ std::cout << "P:\n" << P << std::endl;
                // Create unscented distribution
                detail::unscented<number_of_states, scalar>(
                    XX,
                    filter.x.template block<number_of_states, 1>(starting_state, 0),
                    filter.P.template block<number_of_states, number_of_states>(starting_state, starting_state),
                    alpha*std::sqrt(number_of_states+kappa)
                ); // Step one complete
                const scalar lambda = alpha*alpha*(number_of_states+kappa) - number_of_states;
                const scalar norm = 1/(2*alpha*alpha*(number_of_states+kappa)); // 1/[2*(L+lambda)]

                // Predict points of XX using supplied predict model
                    // State covariance
                for (std::size_t i = 0; i < (2*number_of_states+1); ++i) {
                    fXX.col(i) = f.f( XX.col(i) , u );
                }


                // Mean of predicted distribution: x
                x = fXX.col(0) * 2 * lambda;
                for (std::size_t i = 1; i < (2*number_of_states+1); ++i) {
                    x += fXX.col(i);
                }
                x *= norm; // Step 2 complete. x now \hat{x}_{k+1|k}


                // Covariance of distribution: X
                    // Subtract mean from each point in fXX

                // Center point, premult here by 2 for efficency
                {
                    fXX.col(0) -= x;
                    const scalar w0 = 2.0*(lambda + (1+beta-alpha*alpha)*(alpha*alpha*(number_of_states+kappa)));
                    P.noalias() = fXX.col(0)*fXX.col(0).transpose()*w0;
                }
                // Remaining unscented points
                for (std::size_t i = 1; i < (2*number_of_states+1); ++i) {
                    fXX.col(i) -= x;
                    P.noalias() += fXX.col(i)*fXX.col(i).transpose();
                }
                P *= norm; // Step 3 complete. P now P_{k+1|k}

                // Addative Noise Prediction.
                // Originally computed about center point, but I figure x is the best estimate..
                filter.x.template block<number_of_states, 1>(starting_state, 0) = x;
                filter.P.template block<number_of_states, number_of_states>(starting_state, starting_state) = P + f.Q( x , u );


                //~ std::cout << "Post-time update::::" << std::endl;
                //~ std::cout << "x:\n" << x << std::endl;
                //~ std::cout << "P:\n" << P << std::endl;
            }


            /**
             * Perform a measurement update by comparing the measurements
             * with predicted measurements from different sigma points from
             * the unscented transform and interpret the result.
             * @param h Observation model containing the observation function and measurement covariance
             * @param z Measured states
             * @return void. The internal state of the filter is modified.
             */
            template<int number_of_states, int number_of_controls, int number_of_observations, int starting_state = 0, typename scalar = base_float_t>
            void observe(
                KalmanFilter<number_of_states, number_of_controls, scalar>& filter,
                const Matrix<scalar, number_of_observations, 1>& (*h)(const Matrix<scalar, number_of_states, 1>&),
                const observation<number_of_observations, scalar>& o,
                const scalar alpha = 1e-2, const scalar beta = 2.0, const scalar kappa = 0)
            {
                Matrix<scalar, number_of_states, 2*number_of_states+1> XX;

                Matrix<scalar, number_of_observations, (2*number_of_states+1)> zXX;
                Matrix<scalar, number_of_observations, 1> z_hat;
                Matrix<scalar, number_of_observations,number_of_observations> Pzz;
                Matrix<scalar, number_of_states, number_of_observations> Pxz, K;

                // Create unscented distribution
                detail::template unscented<number_of_states, scalar>(
                    XX,
                    filter.x.template block<number_of_states, 1>(starting_state, 0),
                    filter.P.template block<number_of_states, number_of_states>(starting_state, starting_state),
                    alpha*std::sqrt(number_of_states+kappa)
                ); // Step one complete

                const scalar lambda = alpha*alpha*(number_of_states+kappa) - number_of_states;
                const scalar norm = 1/(2*alpha*alpha*(number_of_states+kappa)); // 1/[2*(L+lambda)]

                // Predict points of XX using supplied observation model
                {
                    for (std::size_t i = 0; i < (2*number_of_states+1); ++i) {
                        zXX.col(i) = h( XX.col(i) );
                    }
                }

                //~ std::cerr << "<zXX: " << std::endl << zXX << ">" << std::endl;

                // Mean of predicted distribution: z_hat
                z_hat = zXX.col(0) * 2 * lambda;
                for (std::size_t i = 1; i < (2*number_of_states+1); ++i) {
                    z_hat += zXX.col(i);
                }
                z_hat *= norm; // Step 4 complete

                //~ std::cerr << "<z_hat: " << std::endl << z_hat << ">" << std::endl;

                // Covariance of observation predict: Xzz
                    // Subtract mean from each point in zXX

                const scalar w0 = 2.0*(lambda + (1+beta-alpha*alpha)*(alpha*alpha*(number_of_states+kappa)));
                // Remaining unscented points
                // Center point, premult here by 2 for efficency
                {
                    Pzz.noalias() = (zXX.col(0)-z_hat)*(zXX.col(0)-z_hat).transpose()*w0;
                }
                for (std::size_t i = 1; i < (2*number_of_states+1); ++i) {
                    zXX.col(i) -= zXX.col(0);
                    Pzz.noalias() += zXX.col(i)*zXX.col(i).transpose();
                }
                zXX.col(0) -= z_hat;
                Pzz *= norm; // Step 5 complete (see below)

                //~ std::cerr << "<Pzz: " << std::endl << Pzz << ">" << std::endl;

                // Innovation covariance
                Pzz += o.R; // Step 5 with extras complete: Pvv

                //~ std::cerr << "<Pvv: " << std::endl << Pzz << ">" << std::endl;
                // Correlation of state with observation: Pxz
                    // Center point, premult here by w0 for efficency
                {
                    Pxz.noalias() = (XX.col(0)-filter.x)*zXX.col(0).transpose()*w0;
                }
                // Remaining unscented points
                for (std::size_t i = 1; i < (2*number_of_states+1); ++i) {
                    Pxz.noalias() += (XX.col(i)-XX.col(0))*zXX.col(i).transpose();
                }
                Pxz *= norm; // Step 6 complete

                //~ std::cerr << "<Pxz: " << std::endl << Pxz << ">" << std::endl;
                // Kalman gain
                K.noalias() = (Pzz.transpose().ldlt().solve(Pxz.transpose())).transpose();

                //~ std::cerr << "<K: " << std::endl << K << ">" << std::endl;
                // Filter update
                filter.x.template block<number_of_states, 1>(starting_state, 0) += K*(o.z - z_hat);
                filter.P.template block<number_of_states, number_of_states>(starting_state, starting_state) -= K*Pzz*K.transpose();


                //~ std::cout << "Post-Measurement update::::" << std::endl;
                //~ std::cout << "x:\n" << x << std::endl;
                //~ std::cout << "P:\n" << P << std::endl;
                //~ std::cout << "K:\n" << K << std::endl;
                //~ std::cout << "XX:\n" << XX << std::endl;
                //~ std::cout << "zXX:\n" << zXX << std::endl;
                //~ std::cout << "Pzz:\n" << Pzz << std::endl;
                //~ std::cout << "Pxz:\n" << Pxz << std::endl;
            }
        }
    }
}

#endif
