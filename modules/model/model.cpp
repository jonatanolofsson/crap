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
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include "modules/model/model.hpp"
#include "modules/controller/model.hpp"
#include "crap/module.hpp"

namespace CRAP {
    namespace observer {
        using std::sin;
        using std::cos;
        using std::atan2;
        using std::acos;
        using std::abs;
        using std::pow;
        using std::sqrt;
        using math::sign;

        namespace model {
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

            void get_forces_and_moments(const state_vector&, Matrix<model_float_t, 3, 1>&, Matrix<model_float_t, 3, 1>&);

            void setup_statics(const state_vector& x) {
                Quaternion<model_float_t> q(x(state::quaternion_part_real), x(state::quaternion_part_vector[0]), x(state::quaternion_part_vector[1]), x(state::quaternion_part_vector[2]));
                q.normalize();

                bf2ned = q.toRotationMatrix();
                ned2bf = bf2ned.transpose(); // Orthonormal matrix
                //~ std::cout << "BF2NED:::::" << std::endl << bf2ned << std::endl << ":::::::" << std::endl;
            }
            void setup_physics(const state_vector& x) {
                V_rel = ned2bf*(x.segment<3>(state::velocities) - x.segment<3>(state::wind_velocities));
            }
            void calculate_velocity(const state_vector& x) {
                xdot.segment<3>(state::positions) = x.segment<3>(state::velocities);
            }
            void calculate_rotation(const state_vector& x) {
                rot <<
                                    0,              x(state::omega_part[Z]),       -x(state::omega_part[Y]),    x(state::omega_part[X]),
                        -x(state::omega_part[Z]),          0,                       x(state::omega_part[X]),    x(state::omega_part[Y]),
                        -x(state::omega_part[Y]),   x(state::omega_part[X]),                0,                  x(state::omega_part[Z]),
                        -x(state::omega_part[X]),  -x(state::omega_part[Y]),       -x(state::omega_part[Z]),           0;
                xdot.segment<4>(state::quaternion) = 0.5*rot*x.segment<4>(state::quaternion);
            }
            void calculate_accelerations(const state_vector& x) {
                Matrix<model_float_t, 3, 1> F, M;
                get_forces_and_moments(x,F,M);
                xdot.segment<3>(state::velocities) = bf2ned*F/parameters::m;
                xdot.segment<3>(state::omega) = I_G.solve(M);
                //~ std::cout << "a: " << F.transpose()/parameters::m << "\t Aned: " << (bf2ned*F/parameters::m).transpose() << "\tM: " << M.transpose() << "\tW: " << I_G.solve(M).transpose() << std::endl;
            }
            void calculate_rotor_velocities(const state_vector& x, const control_vector& u) {
                xdot.segment<4>(state::rotor_velocities) = (u - x.segment<4>(state::rotor_velocities)) / parameters::tau_rotor;
            }
            void calculate_wind_acceleration() {
                xdot.segment<3>(state::wind_velocities).setZero();
            }
            void calculate_gyrodrift() {
                xdot.segment<3>(state::gyro_drifts).setZero();
            }

            namespace rotor {
                Matrix<model_float_t, 3, 1> V_ri, Pri3, V;
                model_float_t mu_ri, mu_ri22, omega_i, alpha_i, v_1i, lambda_i, alpha_si, Vnorm, C_T;

                template<int i> void setup_rotor_physics(const state_vector& x) {
                    new (&D) Map<const Matrix<model_float_t, 3, 1>>(parameters::D[i]);
                    omega_i = x(state::rotor_omega[i]);
                    alpha_i = xdot(state::rotor_omega[i]);
                    V_ri = V_rel + x.segment<3>(state::omega).cross(D);
                    V = V_ri;
                    V.z() = 0;
                    Vnorm = V.norm();

                    if(abs(omega_i) < 1.0) {
                        omega_i = sign(omega_i);
                    }

                    mu_ri = Vnorm/(omega_i * parameters::R);
                    mu_ri22 = SQR(mu_ri) * 0.5;

                    // angle between shaft plane and path (rel. to wind) pp.160
                    model_float_t Vrinorm = V_ri.norm();
                    if(Vrinorm > 1e-4) {
                        alpha_si = M_PI_2 - acos( - V_ri.normalized().dot(Matrix<model_float_t, 3, 1>::UnitZ()) );
                    } else {
                        alpha_si = 0;
                    }
                        //FIXME
                        alpha_si = 0;

                    model_float_t vsquared = SQR(Vnorm);
                    v_1i = std::sqrt(
                        0.5 * (
                            -vsquared
                            + std::sqrt(
                                vsquared
                                +  SQR(parameters::m * parameters::g / (parameters::rho * parameters::A_rotordisk))
                            )
                        )
                    );

                    // FIXME: Check signs
                    lambda_i = mu_ri * alpha_si - v_1i / (omega_i * parameters::R);
                    C_T = 0.25 * parameters::sigma * parameters::a * (
                          ( 2./3. + SQR(mu_ri) ) * parameters::theta0[i]
                        - (0.5 + mu_ri22) * parameters::theta_twist[i]
                        - lambda_i );
                    C_T /= 10;
                    //~ std::cout << i << ": C_T: " << C_T << "\tWi: " << omega_i << "\tlambda_i: " << lambda_i << "\tv_1i: " << v_1i << "\tV_ri: " << V_ri.transpose() << "\tmu_ri: " << mu_ri << "\talpha_si: " << alpha_si << "\tV2: " << vsquared << std::endl;
                }
                template<int i> void calculate_thrust(const state_vector& x, Matrix<model_float_t, 3, 1>& F, Matrix<model_float_t, 3, 1>& M) {
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
                                (((-16*x(state::omega_part[Y])/gamma + x(state::omega_part[X]))/omega_i) / (1 - mu_ri22)),
                                (((-16*x(state::omega_part[X])/gamma + x(state::omega_part[Y]))/omega_i) / (1 + mu_ri22))
                        ).finished();
flapping_angles.setZero();
                    model_float_t a1s = flapping_angles(0); model_float_t b1s = flapping_angles(1);

                    Pri3 <<
                        -sin(a1s),
                        -cos(a1s)*sin(b1s),
                        -cos(a1s)*cos(b1s);


                    model_float_t z = parameters::rotor_height_over_ground - x(state::position[Z]);
                    model_float_t IGE = 1/(1 - SQR(parameters::R/(4*z)));

                    Matrix<model_float_t, 3, 1> F_ri = IGE * C_T * parameters::rho * parameters::A_rotordisk * SQR(parameters::R) * SQRSGN(omega_i) * Pri3;
                    //~ std::cout << i << ": F_ri: " << F_ri.transpose() << "\tFlapping: " << flapping_angles.transpose() << "\tWi: " << omega_i << "\tC_T: " << C_T << "\tV_ri: " << V_ri.transpose() << "\tM: " << D.cross(F_ri).transpose() << std::endl;

                    F += F_ri;
                    M += D.cross(F_ri); // M_ri
                }
                template<int i> void calculate_torque(Matrix<model_float_t, 3, 1>& M) {
                    //~ model_float_t C_Q = C_T*lambda_i; // This is true only with the approximated expression of lambda_i as sqrt(C_T/2) and the approximation C_Q = C_T^1.5/sqrt(2)
                    model_float_t C_Q = parameters::sigma * parameters::a * (
                        (1/(8*parameters::a))*( 1 + 2*mu_ri22 ) * parameters::C_D_rotor
                        + lambda_i * (
                            (1/6)*parameters::theta0[i]
                            - (1/8)*parameters::theta_twist[i]
                            - (1/4)*lambda_i
                            )
                        );
                    Matrix<model_float_t, 3, 1> MT = -(C_Q * parameters::rho * parameters::A_rotordisk * CUBE(parameters::R) * SQRSGN(omega_i)) * Matrix<model_float_t, 3, 1>::UnitZ();
                    M += MT;
                    //~ std::cout << i << ": M: " << MT.transpose() << "\tC_Q: " << C_Q << "\tMc: " << parameters::I_rotor * alpha_i << std::endl;

                    // Counter-torque
                    M.z() += parameters::I_rotor * alpha_i;
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
                        1./6. * parameters::theta0[i] - 0.125 * parameters::theta_twist[i] - 0.125 * lambda_i
                    );
                    M -= C_RM * parameters::rho * parameters::A_rotordisk * CUBE(parameters::R) * SQRSGN(omega_i) * V / Vnorm;
                    //~ std::cout << i << ": Mrm: " << (C_RM * parameters::rho * parameters::A_rotordisk * CUBE(parameters::R) * SQRSGN(omega_i) * V / Vnorm).transpose() << std::endl;
                }
                template<int i> void calculate_hubforces(Matrix<model_float_t, 3, 1>& F, Matrix<model_float_t, 3, 1>& M) {
                    if(Vnorm < 1e-4) return;
                    model_float_t C_H = 0.25 * parameters::sigma * parameters::a * (
                            mu_ri * parameters::C_D_rotor / parameters::a
                            + lambda_i * mu_ri * ( parameters::theta0[i] - 0.5 * parameters::theta_twist[i] )
                        );
                    Matrix<model_float_t, 3, 1> F_hub = C_H * parameters::rho *  parameters::A_rotordisk * SQR( parameters::R * omega_i ) * V / Vnorm;
                    //~ std::cout << i << ": F_hub: " << F_hub.transpose() << ", V: " << V.transpose() << ", M: " << D.cross(F_hub).transpose() << ", C_H: " << C_H << ", lambda_i: " << lambda_i << ", mu_ri: " << mu_ri << std::endl;
                    F += F_hub;
                    M += D.cross(F_hub);
                }
                template<int i> void calculate_propeller_gyroeffect(const state_vector& x, Matrix<model_float_t, 3, 1>& M) {
                    M.x() += parameters::I_rotor * x(state::omega_part[Y]) * omega_i;
                    M.y() += parameters::I_rotor * x(state::omega_part[X]) * omega_i;
                    //~ std::cout << i << ": M: " << (Matrix<model_float_t, 2, 1>() <<
                        //~ parameters::I_rotor * x(state::omega_part[Y]) * omega_i,
                        //~ parameters::I_rotor * x(state::omega_part[X]) * omega_i).finished().transpose() << std::endl;
                }
                template<int i> void get_forces_and_moments(const state_vector& x, Matrix<model_float_t, 3, 1>& F, Matrix<model_float_t, 3, 1>& M) {
                    setup_rotor_physics<i>(x);
                    calculate_thrust<i>(x,F,M);
                    calculate_torque<i>(M);
                    calculate_rotor_windmoment<i>(F,M);
                    calculate_rolling_moment<i>(M);
                    calculate_hubforces<i>(F,M);
                    calculate_propeller_gyroeffect<i>(x,M);
                }
            }

            void calculate_gyroscopic_moments(Matrix<model_float_t, 3, 1>& M) {
                using math::index;
                M.x() += xdot(state::omega_part[Y]) * xdot(state::omega_part[Z]) * ( parameters::I_G[index<0,3>(Y,Y)] - parameters::I_G[index<0,3>(Z,Z)] );
                M.x() += xdot(state::omega_part[X]) * xdot(state::omega_part[Z]) * ( parameters::I_G[index<0,3>(Z,Z)] - parameters::I_G[index<0,3>(X,X)] );
                M.x() += xdot(state::omega_part[X]) * xdot(state::omega_part[Y]) * ( parameters::I_G[index<0,3>(X,X)] - parameters::I_G[index<0,3>(Y,Y)] );
            }

            void calculate_gravity(Matrix<model_float_t, 3, 1>& F) {
                Matrix<model_float_t, 3, 1> F_G = ned2bf * parameters::g * parameters::m * Matrix<model_float_t, 3, 1>::UnitZ();
                F += F_G;
                //~ std::cout << "F_G: " << F_G.transpose() << std::endl;
            }

            void get_forces_and_moments(const state_vector& x, Matrix<model_float_t, 3, 1>& F, Matrix<model_float_t, 3, 1>& M) {
                F.setZero();
                M.setZero();
                setup_statics(x);
                setup_physics(x);

                rotor::get_forces_and_moments<0>(x,F,M);
                rotor::get_forces_and_moments<1>(x,F,M);
                rotor::get_forces_and_moments<2>(x,F,M);
                rotor::get_forces_and_moments<3>(x,F,M);

                //~ calculate_gyroscopic_moments(M);
                calculate_gravity(F);
            }

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

                //~ std::cout << x.segment<3>(state::velocities).transpose() << " :: " << x.segment<3>(state::omega).transpose() << std::endl;
                calculate_velocity(x);
                calculate_rotation(x);
                calculate_rotor_velocities(x,u); // Before accelerations
                calculate_accelerations(x);
                calculate_wind_acceleration();
                calculate_gyrodrift();

                //~ std::cout << "Dot: " << xdot.transpose() << std::endl;
                if((xdot.array()!=xdot.array()).any()) {
                    std::cout << "NaNs!!!" << std::endl;
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
                xnext = x + T*fc(x,u);
                xnext.segment<4>(state::quaternion).normalize();



                //~ inout << x.transpose() << std::endl;
                //~ xnout << xnext.transpose() << std::endl;
                //~ uout << u.transpose() << std::endl;
                //~ dotout << xdot.transpose() << std::endl;

                return xnext;
            }
        }
    }
}
