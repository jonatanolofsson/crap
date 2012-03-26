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
#include "crap/base_types.hpp"
#include <cmath>

namespace CRAP {
    namespace observer {
        using namespace Eigen;
        namespace model {
            #ifndef MODEL_FLOAT_T
            #define MODEL_FLOAT_T
                typedef base_float_t model_float_t;
            #endif
            namespace state {
                typedef base_float_t scalar;
                const int velocity[] = {0,1,2};
                const int velocities = velocity[0];
                const int omega_part[] = {3,4,5};
                const int omega = omega_part[0];
                const int rotor_omega[] = {6,7,8,9};
                const int rotor_velocities = rotor_omega[0];
                const int quaternion_part_vector[] = {10,11,12};
                const int quaternion_part_real = 13;
                const int quaternion = quaternion_part_vector[0];
                const int position[] = {14,15,16};
                const int positions = position[0];
                const int wind_velocity[] = {17,18,19};
                const int wind_velocities = wind_velocity[0];
                const int gyro_drift[] = {20,21,22};
                const int gyro_drifts = gyro_drift[0];
            }

            enum {
                X = 0,
                Y = 1,
                Z = 2
            };

            namespace parameters {
                typedef base_float_t param_t;
                const param_t a = 2 * M_PI;
                const param_t blade_chord = 0.03;
                const param_t C_D_rotor = 0.01;
                //~ const param_t C_T0 = 0.0047;
                const param_t d = 0.215;     // From linkquadDyn.m
                const param_t g = 9.82331;   // http://www.wolframalpha.com/input/?i=gravity+in+link%C3%B6ping
                const param_t h = 0.05;
                const param_t I_rotor = 2.4*10e-6;
                const param_t m = 1.2;
                const param_t R = 0.14;
                const param_t rho = 1.204;   // http://www.wolframalpha.com/input/?i=air+density+at+room+temperature
                const param_t rotor_height_over_ground = 0.25;
                const param_t tau_rotor = 0.1;
                const param_t theta0[] = {
                    -15 * M_PI / 180,
                    15 * M_PI / 180,
                    -15 * M_PI / 180,
                    15 * M_PI / 180
                };
                const param_t theta_twist[] = {
                    10 * M_PI / 180,
                    -10 * M_PI / 180,
                    10 * M_PI / 180,
                    -10 * M_PI / 180
                };
                const param_t theta_tip[] = {
                    -5 * M_PI / 180,
                    5 * M_PI / 180,
                    -5 * M_PI / 180,
                    5 * M_PI / 180
                };

                const param_t A_rotordisk = M_PI * R * R;
                const param_t A_blade = 2 * R * blade_chord;
                const param_t sigma = A_blade / A_rotordisk;


                const param_t D[4][3] = {
                    {d,  0, -h},
                    {0, -d, -h},
                    {-d, 0, -h},
                    {0,  d, -h}
                };
                const param_t I_G[9] = {
                    0.009561,       0.0,            0.0,
                    0.0,            0.009855,       0.0,
                    0.0,            0.0,            0.0152
                };
                const param_t A_body[9] = {
                    0.01, 0.0,  0.0,
                    0.0,  0.01, 0.0,
                    0.0,  0.0,  0.011
                };
                const param_t C_D[9] = {
                    1.0,  0.0,  0.0,
                    0.0,  1.0,  0.0,
                    0.0,  0.0,  1.1
                };
            }

            namespace control {
                const int velocity = 0;
            }

            using parameters::param_t;
            const param_t frequency                  = 30.0; ///< Hz
            const param_t T = 1.0/frequency;
            const int number_of_states              = 23;
            const int number_of_controls            = 4;
            const int number_of_rotors              = 4;

            typedef Matrix<state::scalar, number_of_states, 1> state_vector;
            typedef Matrix<state::scalar, number_of_controls, 1> control_vector;
            typedef Matrix<state::scalar, number_of_states, number_of_states> state_covariance_matrix;

            const state_vector& fc(const state_vector& x, const control_vector& u);
            const state_vector& f(const state_vector& x, const control_vector& u);
        }
    }
}

#endif
