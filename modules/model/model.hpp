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
#include <Eigen/Geometry>
#include "crap/base_types.hpp"
#include <cmath>
#include "math/math.hpp"


#include <iostream>
inline std::ostream& operator<<(std::ostream& o, const Eigen::Quaternion<double>& q) {
    return (o << q.vec().transpose() << " " << q.w());
}

namespace CRAP {
    using namespace Eigen;
    namespace model {
        #ifndef MODEL_FLOAT_T
        #define MODEL_FLOAT_T
            typedef base_float_t model_float_t;
        #endif

        void enable_controller();
        void disable_controller();

        const int number_of_states              = 23;
        const int number_of_controls            = 4;
        const int number_of_rotors              = 4;

        namespace state {
            typedef base_float_t scalar;
            const int velocities[]              = {0,1,2};
            const int velocity                  = velocities[0];
            const int omega[]                   = {3,4,5};
            const int Omega                     = omega[0];
            const int omega_r[]                 = {6,7,8,9};
            const int rotor_velocities          = omega_r[0];
            const int quaternion_part_vector[]  = {10,11,12};
            const int quaternion_part_real      = 13;
            const int quaternion                = quaternion_part_vector[0];
            const int positions[]               = {14,15,16};
            const int position                  = positions[0];
            const int gyro_offsets[]            = {17,18,19};
            const int gyro_offset               = gyro_offsets[0];
            const int wind_velocities[]         = {20,21,22};
            const int wind_velocity             = wind_velocities[0];
            //~ const int qPw_vector[]              = {23,24,25};
            //~ const int qPw_real                  = 26;
            //~ const int qPw                       = qPw_vector[0];
            //~ const int camera_origins[]          = {27,28,29};
            //~ const int camera_origin             = camera_origins[0];
            //~ const int camera_scale              = 23;
            //~ const int accelerations[]           = {31,32,33};
            //~ const int acceleration              = accelerations[0];
            //~ const int accelerometer_biases[]    = {24,25,26};
            //~ const int accelerometer_bias        = accelerometer_biases[0];
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
                -8 * M_PI / 180,
                8 * M_PI / 180,
                -8 * M_PI / 180,
                8 * M_PI / 180
            };
            const param_t theta_twist[] = {
                7 * M_PI / 180,
                -7 * M_PI / 180,
                7 * M_PI / 180,
                -7 * M_PI / 180
            };
            const param_t theta_tip[] = {
                -1 * M_PI / 180,
                1 * M_PI / 180,
                -1 * M_PI / 180,
                1 * M_PI / 180
            };

            const param_t A_rotordisk = M_PI * R * R;
            const param_t A_blade = 2 * R * blade_chord;
            const param_t sigma = A_blade / A_rotordisk;

            const param_t wind_decay = 0.01;

            //~ const param_t magnetic_inclination[] = { // From http://www.wolframalpha.com/input/?i=Magnetic+inclination+in+Link%C3%B6ping
                //~ 15.7,
                //~ 1.11,
                //~ 48.4
            //~ }

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

            namespace simple {
                const param_t a = (1.4517e-5)*SQR(2*M_PI/60);
                const param_t d = 2.93e-9;
            }
        }

        using parameters::param_t;
        const param_t frequency                  = 30.0; ///< Hz
        const param_t T = 1.0/frequency;

        typedef Matrix<state::scalar, number_of_states, 1> state_vector;
        typedef Matrix<state::scalar, number_of_controls, 1> control_vector;
        typedef Matrix<state::scalar, number_of_states, number_of_states> state_covariance_matrix;

        const state_vector& fc(const state_vector& x, const control_vector& u);
        const state_vector& f(const state_vector& x, const control_vector& u);

        namespace imu {
            typedef state::scalar scalar;
            const int accelerometers[]          = {0,1,2};
            const int accelerometer             = accelerometers[0];
            const int gyroscopes[]              = {3,4,5};
            const int gyroscope                 = gyroscopes[0];
            const int pressure                  = 6;
            const int magnetometers[]           = {7,8,9};
            const int magnetometer              = magnetometers[0];
            const int battery                   = 10;

            const int data_size = 6;
            typedef Matrix<scalar, data_size, 1> measurement_t;
            const measurement_t& measurement(const state_vector& x);

            namespace parameters {
                const param_t L                 = 0.0065;
                const param_t M                 = 0.0289644;
                const param_t p0                = 100030.4;//99872; //101325;
                const param_t R                 = 8.31447;
                const param_t T0                = 288.15;

                const param_t magnetic_inclination[] = { // Normalized, from http://www.wolframalpha.com/input/?i=Magnetic+inclination+in+Link%C3%B6ping
                    0.3085,
                    0.0218,
                    0.9510
                };

                const param_t accelerometer_bias[] = {
                    -0.28411,
                    0.3768,
                    -0.0136
                };
            }
        }

        namespace camera {
            typedef state::scalar scalar;
            const int quaternion_part_vector[]  = {0,1,2};
            const int quaternion_part_real      = 3;
            const int quaternion                = quaternion_part_vector[0];
            const int positions[]               = {4,5,6};
            const int position                  = positions[0];

            const int data_size = 7;
            typedef Matrix<scalar, data_size, 1> measurement_t;

            void initialize(
                state_vector& x,
                const Eigen::Matrix<scalar, 3, 1>& XPTAM,
                const Eigen::Quaternion<double>& qPTAM,
                const Eigen::Quaternion<double>& qbc);

            void set_qbc(const Eigen::Quaternion<scalar>& qbc);
            const measurement_t& measurement(const state_vector&);

            namespace parameters {
                const param_t position[]        = {0, 0, 0.1};
                const Map<const Matrix<param_t, 3, 1> > camera_position(position);
                const param_t tilt              = -30*M_PI/180;

                const param_t quaternion_covariance     = 0.01;
                const param_t origin_covariance         = 0.01;
                const param_t scale_covariance          = 0.01;
            }

            enum {
                REINITIALIZE_BIT = 6,
                REINITIALIZE_SCALE_BIT = 7
            };
        }
    }
}

#endif
