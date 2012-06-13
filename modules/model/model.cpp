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

#include "math/math.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include "modules/model/model.hpp"
#include "modules/controller/model.hpp"
#include "crap/module.hpp"
#include "crap/config.hpp"

namespace CRAP {
    namespace model {
        using std::sin;
        using std::cos;
        using std::atan2;
        using std::acos;
        using std::abs;
        using std::pow;
        using std::sqrt;
        using math::sign;
        using std::min;

        bool controller_is_active = false;

        void enable_controller() { controller_is_active = true; }
        void disable_controller() { controller_is_active = false; }

        Map<Matrix<param_t, 3, number_of_rotors> > Dmat((param_t*)parameters::D);
        Map<Matrix<param_t, 3, 1> > D((param_t*)parameters::D);
        Map<Matrix<param_t, 3, 3, RowMajor> > I_Gdata((param_t*)parameters::I_G);
        LDLT<Matrix<param_t, 3, 3> > I_G(I_Gdata);
        Map<Matrix<param_t, 3, 3, RowMajor> > A_body((param_t*)parameters::A_body);
        Map<Matrix<param_t, 3, 3, RowMajor> > C_D((param_t*)parameters::C_D);

        state_vector xdot,xnext; ///< Internal storage for state calculation
        Matrix<model_float_t, 3, 1> V_rel;
        Matrix<model_float_t, 3, 3> ned2bf, bf2ned;
        // Rotation
        Matrix<model_float_t, 4, 4> rot;

        void get_forces_and_moments(const state_vector&, Matrix<model_float_t, 3, 1>&, Matrix<model_float_t, 3, 1>&,bool=true);

        void setup_statics(const state_vector& x) {
            Quaternion<model_float_t> qwb(
                x(state::quaternion_part_real),
                x(state::quaternion_part_vector[0]),
                x(state::quaternion_part_vector[1]),
                x(state::quaternion_part_vector[2])
            );
            qwb.normalize();

            bf2ned = qwb.toRotationMatrix(); // q^{wb}
            ned2bf = bf2ned.transpose(); // Orthonormal matrix
            //~ std::cout << "BF2NED:::::" << std::endl << bf2ned << std::endl << ":::::::" << std::endl;
            //~ std::cout << "NED2BF:::::" << std::endl << ned2bf << std::endl << ":::::::" << std::endl;
        }
        void setup_physics(const state_vector& x) {
            V_rel = ned2bf*(x.segment<3>(state::velocity) - x.segment<3>(state::wind_velocity));
            //~ std::cout << "V_rel: " << V_rel.transpose() << "\t NED velocity: " << x.segment<3>(state::velocity).transpose() << "\t NED wind: " << x.segment<3>(state::wind_velocity).transpose() << std::endl;
        }
        void calculate_velocity(const state_vector& x) {
            xdot.segment<3>(state::position) = x.segment<3>(state::velocity);
        }
        void calculate_rotation(const state_vector& x) {
            rot <<
                                0,        -x(state::omega[Z]),        x(state::omega[Y]),    x(state::omega[X]),
                     x(state::omega[Z]),          0,                 -x(state::omega[X]),    x(state::omega[Y]),
                    -x(state::omega[Y]),   x(state::omega[X]),                0,             x(state::omega[Z]),
                    -x(state::omega[X]),  -x(state::omega[Y]),       -x(state::omega[Z]),           0;
            xdot.segment<4>(state::quaternion) = 0.5*rot*x.segment<4>(state::quaternion);
        }
        void calculate_accelerations(const state_vector& x) {
            Matrix<model_float_t, 3, 1> F, M;
            get_forces_and_moments(x,F,M);
            xdot.segment<3>(state::velocity) = bf2ned*F/parameters::m;
            xdot.segment<3>(state::Omega) = bf2ned*I_G.solve(M);
            //~ std::cout << "a: " << F.transpose()/parameters::m << "\t Aned: " << (bf2ned*F/parameters::m).transpose() << "\tM: " << M.transpose() << "\tW: " << I_G.solve(M).transpose() << std::endl;
            //~ std::cout << "a: " << F.transpose()/parameters::m << "\t Aned: " << (bf2ned*F/parameters::m).transpose() << "\tV_rel: " << V_rel.transpose() << "\tM: " << M.transpose() << "\tW: " << I_G.solve(M).transpose() << std::endl;
        }
        void calculate_rotor_velocities(const state_vector& x, const control_vector& u) {
            if(controller_is_active) {
                xdot.segment<4>(state::rotor_velocities) = (u - x.segment<4>(state::rotor_velocities)) / parameters::tau_rotor;
            } else {
                xdot.segment<4>(state::rotor_velocities).setZero();
            }
        }
        void calculate_wind_acceleration(const state_vector& x) {
            //~ xdot.segment<3>(state::wind_velocity).setZero();
            xdot.segment<3>(state::wind_velocity) = -model::parameters::wind_decay*x.segment<3>(state::wind_velocity);
        }
        void calculate_gyrooffset() {
            xdot.segment<3>(state::gyro_offset).setZero();
        }

        namespace rotor {
            Matrix<model_float_t, 3, 1> V_ri, Pri3, V;
            model_float_t mu_ri, mu_ri22, omega_i, alpha_i, v_1i, lambda_i, alpha_si, Vnorm, C_T;

            template<int i> void setup_rotor_physics(const state_vector& x) {
                new (&D) Map<const Matrix<model_float_t, 3, 1>>(parameters::D[i]);
                omega_i = x(state::omega_r[i]);
                #ifdef EXTENDED_MODEL
                    alpha_i = xdot(state::omega_r[i]);
                    V_ri = V_rel + x.segment<3>(state::Omega).cross(D);
                    V = V_ri;
                    V.z() = 0;
                    Vnorm = V.norm();

                    if(abs(omega_i) < 1.0) {
                        omega_i = sign(omega_i);
                    }

                    mu_ri = Vnorm/(omega_i * parameters::R);
                    mu_ri22 = SQR(mu_ri) * 0.5;

                    // angle between shaft plane and path (rel. to wind) pp.160
                    //~ model_float_t Vrinorm = V_ri.norm();
                    if(Vnorm > 1e-4) {
                        alpha_si = M_PI_2 - acos( - V_ri.normalized().dot(Matrix<model_float_t, 3, 1>::UnitZ()) );
                    } else {
                        alpha_si = 0;
                    }
                        //FIXME
                        alpha_si = 0;

                    model_float_t vsquared = SQR(Vnorm);
                    v_1i = std::sqrt(std::max(0.0,
                        0.5 * (
                            std::sqrt(
                                vsquared
                                +  SQR(parameters::m * parameters::g / (parameters::rho * parameters::A_rotordisk))
                            )
                            - vsquared
                        )
                    ));

                    // FIXME: Check signs
                    lambda_i = mu_ri * alpha_si - v_1i / (omega_i * parameters::R);
                    C_T = 0.25 * parameters::sigma * parameters::a * (
                          ( 2./3. + SQR(mu_ri) ) * parameters::theta0[i]
                        - (0.5 + mu_ri22) * parameters::theta_twist[i]
                        + lambda_i );
                    //~ std::cout << i << ": C_T: " << C_T << "\tWi: " << omega_i << "\tlambda_i: " << lambda_i << "\tv_1i: " << v_1i << "\tV_ri: " << V_ri.transpose() << "\tmu_ri: " << mu_ri << "\talpha_si: " << alpha_si << "\tV2: " << vsquared << "\t root of: " <<
                        //~ 0.5 * (
                            //~ std::sqrt(
                                //~ vsquared
                                //~ +  SQR(parameters::m * parameters::g / (parameters::rho * parameters::A_rotordisk))
                            //~ )
                            //~ - vsquared
                        //~ ) << std::endl;
                    #endif
            }
            template<int i> void calculate_thrust(const state_vector& x, Matrix<model_float_t, 3, 1>& F, Matrix<model_float_t, 3, 1>& M) {
                #ifndef EXTENDED_MODEL
                    Matrix<model_float_t, 3, 1> F_ri; F_ri << 0, 0, -parameters::simple::a * SQR(omega_i);
                    F += F_ri;
                    M += D.cross(F_ri); // M_ri
                #else
                    if(std::abs(omega_i) < 10) { // Some delta
                        Pri3 = -Matrix<model_float_t, 3, 1>::UnitZ();
                        return;
                    }

                    model_float_t psi = atan2(V_ri(Y), V_ri(X));

                    static const model_float_t gamma = parameters::rho * parameters::a * parameters::blade_chord * POW4(parameters::R) / parameters::I_rotor;

                    Matrix<model_float_t, 2, 1> flapping_angles = Rotation2D<model_float_t>(psi)
                            * (Matrix<model_float_t, 2, 1>() <<
                                (mu_ri * (4 * parameters::theta_twist[i] - 2*lambda_i) / (1 - mu_ri22)),
                                ((4/3) * (C_T * 2 * mu_ri * gamma / (parameters::sigma * 3 * parameters::a) + mu_ri) / (1 + mu_ri22))
                            ).finished()
                        + (Matrix<model_float_t, 2, 1>() <<
                                (((-16*x(state::omega[Y])/gamma + x(state::omega[X]))/omega_i) / (1 - mu_ri22)),
                                (((-16*x(state::omega[X])/gamma + x(state::omega[Y]))/omega_i) / (1 + mu_ri22))
                        ).finished();
    flapping_angles.setZero(); //FIXME
                    model_float_t a1s = flapping_angles(0); model_float_t b1s = flapping_angles(1);

                    Pri3 <<
                        -sin(a1s),
                        -cos(a1s)*sin(b1s),
                        -cos(a1s)*cos(b1s);


                    model_float_t z = parameters::rotor_height_over_ground - min(0.0, x(state::positions[Z]));
                    model_float_t IGE = 1/(1 - SQR(parameters::R/(4*z)));
                    //~ IGE = 1.0;

                    Matrix<model_float_t, 3, 1> F_ri = IGE * C_T * parameters::rho * parameters::A_rotordisk * SQR(parameters::R) * SQRSGN(omega_i) * Pri3;
                    //~ std::cout << i << ": F_ri: " << F_ri.transpose() << "\tFlapping: " << flapping_angles.transpose() << "\tWi: " << omega_i << "\tC_T: " << C_T << "\tV_ri: " << V_ri.transpose() << "\tM: " << D.cross(F_ri).transpose() << std::endl;

                    F += F_ri;
                    M += D.cross(F_ri); // M_ri
                #endif
            }
            template<int i> void calculate_torque(Matrix<model_float_t, 3, 1>& M) {
                #ifndef EXTENDED_MODEL
                    Matrix<model_float_t, 3, 1> Q; Q << 0, 0, -parameters::simple::d * SQRSGN(omega_i);
                    M += Q; // M_ri
                #else
                    model_float_t C_Q = parameters::sigma * parameters::a * (
                        (1/(8*parameters::a))*( 1 + 2*mu_ri22 ) * parameters::C_D_rotor
                        + lambda_i * (
                            (1/6)*parameters::theta0[i]
                            - (1/8)*parameters::theta_twist[i]
                            + (1/4)*lambda_i
                            )
                        );
                    Matrix<model_float_t, 3, 1> Q = -(C_Q * parameters::rho * parameters::A_rotordisk * CUBE(parameters::R) * SQRSGN(omega_i)) * Matrix<model_float_t, 3, 1>::UnitZ();
                    M += Q;
                    //~ std::cout << i << ": Q: " << Q.transpose() << "\tC_Q: " << C_Q << "\tMc: " << parameters::I_rotor * alpha_i << std::endl;

                    // Counter-torque
                    M.z() += parameters::I_rotor * alpha_i;
                #endif
            }
            template<int i> void calculate_rotor_windmoment(Matrix<model_float_t, 3, 1>& F, Matrix<model_float_t, 3, 1>& M) {
                Matrix<model_float_t, 3, 1> Vpri = V_ri.dot(Pri3)*Pri3;
                Matrix<model_float_t, 3, 1> F_wi = -0.5 * parameters::rho * parameters::C_D_rotor * parameters::A_blade * Vpri * Vpri.norm();
                //~ std::cout << i << ": F_wi: " << F_wi.transpose() << ", V_pri: " << Vpri.transpose() << ", V_ri: " << V_ri.transpose() << std::endl;
                F += F_wi;
                M += D.cross(F_wi);
            }
            template<int i> void calculate_rolling_moment(Matrix<model_float_t, 3, 1>& M) {
                if(Vnorm < 1e-4) return;
                model_float_t C_RM = parameters::sigma * parameters::a * - mu_ri * (
                    1./6. * parameters::theta0[i] - 0.125 * parameters::theta_twist[i] + 0.125 * lambda_i
                );
                M -= C_RM * parameters::rho * parameters::A_rotordisk * CUBE(parameters::R) * SQRSGN(omega_i) * V / Vnorm;
                //~ std::cout << i << ": Mrm: " << (C_RM * parameters::rho * parameters::A_rotordisk * CUBE(parameters::R) * SQRSGN(omega_i) * V / Vnorm).transpose() << std::endl;
            }
            template<int i> void calculate_hubforces(Matrix<model_float_t, 3, 1>& F, Matrix<model_float_t, 3, 1>& M) {
                if(Vnorm < 1e-4) return;
                model_float_t C_H = 0.25 * parameters::sigma * parameters::a * (
                        mu_ri * parameters::C_D_rotor / parameters::a
                        + lambda_i * mu_ri * (
                            parameters::theta0[i]
                            - 0.5 * parameters::theta_twist[i]
                        )
                    );
                Matrix<model_float_t, 3, 1> F_hub = C_H * parameters::rho *  parameters::A_rotordisk * SQR( parameters::R * omega_i ) * V / Vnorm;
                //~ std::cout << i << ": F_hub: " << F_hub.transpose() << ", V: " << V.transpose() << ", M: " << D.cross(F_hub).transpose() << ", C_H: " << C_H << ", lambda_i: " << lambda_i << ", mu_ri: " << mu_ri << std::endl;
                F += F_hub;
                M += D.cross(F_hub);
            }
            template<int i> void calculate_propeller_gyroeffect(const state_vector& x, Matrix<model_float_t, 3, 1>& M) {
                M.x() += parameters::I_rotor * x(state::omega[Y]) * omega_i;
                M.y() += parameters::I_rotor * x(state::omega[X]) * omega_i;
                //~ std::cout << i << ": M: " << (Matrix<model_float_t, 2, 1>() <<
                    //~ parameters::I_rotor * x(state::omega[Y]) * omega_i,
                    //~ parameters::I_rotor * x(state::omega[X]) * omega_i).finished().transpose() << std::endl;
            }
            template<int i> void get_forces_and_moments(const state_vector& x, Matrix<model_float_t, 3, 1>& F, Matrix<model_float_t, 3, 1>& M) {
                setup_rotor_physics<i>(x);
                calculate_thrust<i>(x,F,M);
                calculate_torque<i>(M);
                #ifdef EXTENDED_MODEL
                    calculate_rotor_windmoment<i>(F,M);
                    calculate_rolling_moment<i>(M);
                    calculate_hubforces<i>(F,M);
                    calculate_propeller_gyroeffect<i>(x,M);
                #endif
            }
        }

        void calculate_gyroscopic_moments(Matrix<model_float_t, 3, 1>& M) {
            using math::index;
            M.x() += xdot(state::omega[Y]) * xdot(state::omega[Z]) * ( parameters::I_G[index<0,3>(Y,Y)] - parameters::I_G[index<0,3>(Z,Z)] );
            M.y() += xdot(state::omega[X]) * xdot(state::omega[Z]) * ( parameters::I_G[index<0,3>(Z,Z)] - parameters::I_G[index<0,3>(X,X)] );
            M.z() += xdot(state::omega[X]) * xdot(state::omega[Y]) * ( parameters::I_G[index<0,3>(X,X)] - parameters::I_G[index<0,3>(Y,Y)] );
        }

        void calculate_gravity(Matrix<model_float_t, 3, 1>& F) {
            Matrix<model_float_t, 3, 1> F_G = ned2bf * parameters::g * parameters::m * Matrix<model_float_t, 3, 1>::UnitZ();
            F += F_G;
            //~ std::cout << "F_G: " << F_G.transpose() << std::endl;
        }

        void get_forces_and_moments(const state_vector& x, Matrix<model_float_t, 3, 1>& F, Matrix<model_float_t, 3, 1>& M, bool include_gravity) {
            F.setZero();
            M.setZero();
            setup_statics(x);
            setup_physics(x);

            rotor::get_forces_and_moments<0>(x,F,M);
            rotor::get_forces_and_moments<1>(x,F,M);
            rotor::get_forces_and_moments<2>(x,F,M);
            rotor::get_forces_and_moments<3>(x,F,M);

            //~ calculate_gyroscopic_moments(M); //FIXME: xdot?
            if(include_gravity) calculate_gravity(F);
        }

        //~ void set_camera_origin(const state_vector& x) {
            //~ xdot.segment<4>(state::qPw).setZero();
            //~ xdot.segment<3>(state::camera_origin).setZero();
            //~ xdot(state::camera_scale) = (x(state::camera_scale) > 0.5 ? 0 : 0.001);
        //~ }
        //~ void calculate_accelerometer_bias() {
            //~ xdot.segment<3>(state::accelerometer_bias).setZero();
        //~ }

        /*!
         * \brief   Calculate the derivative of the states
         *
         * This function contains the expressions for the mathematical
         * derivatives of each state.
         * \param   x   Current state
         * \param   u   Current control signal
         * \return  The derivative of the states at the point defined by x and u
         */
        const state_vector& fc(const state_vector& x, const control_vector& u) {
            //~ std::cout << "In: " << x.transpose() << std::endl;

            //~ std::cout << x.segment<3>(state::velocity).transpose() << " :: " << x.segment<3>(state::Omega).transpose() << std::endl;
            calculate_velocity(x);
            calculate_rotation(x);
            calculate_rotor_velocities(x,u); // Before accelerations
            calculate_accelerations(x);
            calculate_wind_acceleration(x);
            calculate_gyrooffset();
            //~ calculate_accelerometer_bias();
            //~ set_camera_origin(x);

            //~ std::cout << "Dot: " << xdot.transpose() << std::endl;
            if((xdot.array()!=xdot.array()).any()) {
                std::cout << "NaNs!!!" << std::endl;
                std::cout << "Dot: " << xdot.transpose() << std::endl;
                while(true) sleep(10000);
            }
            return xdot;
        }


        /*!
         * \brief   Predict the filter's next time step
         *
         * Using the model, current state and the current control signal,
         * the state at the next time-step is predicted using a RK4 method
         * for integrating the state, given the derivatives of the states.
         * \see     fc
         * \param   x   Current state
         * \param   u   Current control signal
         * \return  Prediction of state in the next time step
         */
         //~ std::ofstream inout("x.mat");
         //~ std::ofstream xnout("xnext.mat");
         //~ std::ofstream uout("u.mat");
         //~ std::ofstream dotout("xdot.mat");
        const state_vector& f(const state_vector& x, const control_vector& u) {
            //~ xnext = math::rk4(fc, x, u, T);
            //~ std::cout << "x: " << x.transpose() << "\t u: " << u.transpose() << std::endl;
            xnext = x + T*fc(x,u);
            xnext.segment<4>(state::quaternion).normalize();
            //~ xnext.segment<4>(state::qPw).normalize();
            //~ std::cout << "in: " << x(state::velocities[Z]) << "\tdot: " << xdot(state::velocities[Z]) << "\tdelta: " << T*xdot(state::velocities[Z]) << "\t next: " << xnext(state::velocities[Z]) << std::endl;



            //~ inout << x.transpose() << std::endl;
            //~ xnout << xnext.transpose() << std::endl;
            //~ uout << u.transpose() << std::endl;
            //~ dotout << xdot.transpose() << std::endl;

            return xnext;
        }

        namespace imu {
            const Map<const Matrix<scalar, 3, 1> > magnetic_inclination(model::imu::parameters::magnetic_inclination);
            const Map<const Matrix<scalar, 3, 1> > accelerometer_bias(model::imu::parameters::accelerometer_bias);

            Matrix<model_float_t, imu::data_size, 1> imu_data;
            const Matrix<model_float_t, imu::data_size, 1>& measurement(const state_vector& x) {
                Matrix<model_float_t, 3, 1> F, M;
                get_forces_and_moments(x,F,M,false);

                // Accelerometer
                imu_data.segment<3>(imu::accelerometer) = F/model::parameters::m + accelerometer_bias;// + x.segment<3>(state::accelerometer_bias);

                // Gyroscope
                imu_data.segment<3>(imu::gyroscope) = x.segment<3>(state::Omega) + x.segment<3>(state::gyro_offset);

                // Pressure sensor
                //~ p = p_{0} \left( 1 - \frac{L \cdot h}{T_{0}} \right)^{\frac{g \cdot M}{R\cdot L}}
                //~ imu_data(imu::pressure) = imu::parameters::p0 * std::pow( 1 + imu::parameters::L*x(state::positions[Z])/imu::parameters::T0, model::parameters::g * imu::parameters::M / (imu::parameters::R * imu::parameters::L));

                // Magnetometer
                //~ imu_data.segment<3>(imu::magnetometer) = ned2bf * magnetic_inclination;
                return imu_data;
            }
        }


        namespace camera {
            const Map<const Matrix<camera::scalar, 3, 1> > bf2cam_translation(camera::parameters::position);
            typedef Matrix<model_float_t, camera::data_size, 1> camera_measurement;
            camera_measurement camera_data;
            Quaternion<model_float_t> qPw;
            Eigen::Quaternion<scalar> qbc;
            Matrix<model_float_t, 3, 1> origin;
            double camera_scale;


            void set_qbc(const Eigen::Quaternion<scalar>& qbc_) { qbc = qbc_; }

            const Matrix<model_float_t, camera::data_size, 1>& measurement(const state_vector& x) {
                const Quaternion<scalar> qwb(
                    x(state::quaternion_part_real),
                    x(state::quaternion_part_vector[0]),
                    x(state::quaternion_part_vector[1]),
                    x(state::quaternion_part_vector[2])
                );
                camera_data.segment<4>(camera::quaternion) = (qPw * qwb * qbc).coeffs();
                camera_data.segment<3>(camera::position) = qPw.toRotationMatrix()*(x.segment<3>(state::position) + qwb.toRotationMatrix()*parameters::camera_position - origin)*camera_scale;//x(state::camera_scale);
                //~ std::cout << x.transpose() << std::endl;
                //~ std::cout << camera_data.transpose() << std::endl;
                return camera_data;
            }

            void initialize(
                state_vector& x,
                const Eigen::Matrix<scalar, 3, 1>& XPTAM,
                const Eigen::Quaternion<scalar>& qPTAM,
                const Eigen::Quaternion<scalar>& qbc_)
            {
                set_qbc(qbc_);
                const Quaternion<scalar> qwb(
                    x(state::quaternion_part_real),
                    x(state::quaternion_part_vector[0]),
                    x(state::quaternion_part_vector[1]),
                    x(state::quaternion_part_vector[2])
                );
                //~ Map<Matrix<scalar, 3,1> > origin(&x.data()[state::camera_origin]);
                Matrix<scalar, 3, 1> camera_position = x.segment<3>(state::position) + qwb.toRotationMatrix()*parameters::camera_position;


                std::cout << "::::::::::::: Initializing camera transformation: " << std::endl;
                std::cout << "Camera position: " << camera_position.transpose() << std::endl;
                std::cout << "Quad orientation: " << qwb << std::endl;
                std::cout << "Camera orientation: " << qbc << std::endl;
                std::cout << "Full camera orientation: " << qwb*qbc << std::endl;
                std::cout << "Measured position: " << XPTAM.transpose() << std::endl;
                std::cout << "Measured orientation" << qPTAM << std::endl;

                qPw = qPTAM * qbc.inverse() * qwb.inverse();
                //~ x(state::qPw_real) = qPw.w();
                //~ x(state::qPw_vector[0]) =  qPw.x();
                //~ x(state::qPw_vector[1]) =  qPw.y();
                //~ x(state::qPw_vector[2]) =  qPw.z();

                scalar Xnorm = XPTAM.norm();
                Matrix<double, 3, 1> camera_to_origin = -qPw.toRotationMatrix().transpose() * XPTAM/Xnorm;
                scalar lambda = - camera_position.z() / camera_to_origin.z();

                origin = camera_position + lambda * camera_to_origin;
                camera_scale = Xnorm / std::abs(lambda);
                //~ x(state::camera_scale) = camera_scale;

                std::cout << "PTAM Orientation: " << qPw << std::endl;
                std::cout << "Camera to origin: " << camera_to_origin.transpose() << std::endl;
                std::cout << "Lambda: " << lambda << std::endl;
                std::cout << "Scale: " << camera_scale << std::endl;
                std::cout << "Origin: " << origin.transpose() << std::endl << std::endl;
                std::cout << "Rotation control: " << (qPw.toRotationMatrix() * Eigen::Vector3d::UnitY()).transpose() << std::endl;
                std::cout << "Rotation control: " << (qPw.toRotationMatrix() * Eigen::Vector3d::UnitX()).transpose() << std::endl;
                std::cout << "Rotation control: " << -(qPw.toRotationMatrix() * Eigen::Vector3d::UnitZ()).transpose() << std::endl << std::endl;

                std::cout << "Control: " << measurement(x).transpose() << std::endl;
                //~ Vector3d XPTAMcheck = qPw.toRotationMatrix()*(x.segment<3>(state::position) + qwb.toRotationMatrix()*parameters::camera_position - origin)*x(state::camera_scale);
                //~ std::cout << "Position control" << XPTAMcheck.transpose() << std::endl;
                //~ Quaternion<double> qPTAMcheck(qPw * qwb * qbc);
                //~ std::cout << "Orientation control" << qPTAMcheck << std::endl;
            }
        }
    }
}
