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

#include "crap/module.hpp"
#include "modules/controller/model.hpp"
#include "modules/model/model.hpp"
#include "math/control/LQ.hpp"
#include "math/math.hpp"
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace CRAP {
    namespace controller {
        YAML::Node config;

        #ifdef CRAP_OUTPUT_DATA
            std::ofstream Aout("A.mat");
            std::ofstream Bout("B.mat");
            std::ofstream Qout("Q.mat");
            std::ofstream Rout("R.mat");
            std::ofstream rout("r.mat");
            std::ofstream Lout("L.mat");
            std::ofstream Lrout("Lr.mat");
            std::ofstream eXout("eX.mat");
            std::ofstream uout("u.mat");
        #endif

        #ifdef CRAP_PLOT
            using namespace cpplot;
            static const int count = 30*10;
            auto u0plot = figure("Control signal")->subplot(2,2,1)->title("u0")->add<Line>()->set_capacity(count);
            auto u1plot = figure("Control signal")->subplot(2,2,2)->title("u1")->add<Line>()->set_capacity(count);
            auto u2plot = figure("Control signal")->subplot(2,2,3)->title("u2")->add<Line>()->set_capacity(count);
            auto u3plot = figure("Control signal")->subplot(2,2,4)->title("u3")->add<Line>()->set_capacity(count);
        #endif


        namespace detail {
            control_vector u;
            reference_vector r;

            typedef Matrix<scalar, observer::model::number_of_states, 1> full_state_vector;
            typedef CRAP::control::LQ<model::number_of_states+1, model::number_of_controls, model::number_of_references> controller_type;
            typedef controller_type::controller_model lmodel_t;
            lmodel_t lmodel;
            controller_type::state_vector extended_x;
            controller_type regulator;



            void linearize(const full_state_vector& x0, const control_vector& u) {
                Block<lmodel_t::state_matrix, controller::model::number_of_linearized_states, controller::model::number_of_linearized_states> A(lmodel.A, 0, 0);
                Block<lmodel_t::control_matrix, controller::model::number_of_linearized_states, controller::model::number_of_controls> B(lmodel.B, 0, 0);
                ::CRAP::control::jacobians<controller::model::number_of_linearized_states, controller::model::number_of_controls, 0, 0, scalar>(
                    observer::model::fc, x0, u,
                    A,
                    B
                );

                lmodel.A.block<controller::model::number_of_linearized_states, 1>(0, controller::model::number_of_states)
                    = observer::model::fc(x0, u).segment<controller::model::number_of_linearized_states>(0)
                        - A*x0.segment<controller::model::number_of_linearized_states>(0);
            }

            void calculate_control_signal(const full_state_vector x0) {
                using observer::model::Z;
                linearize(x0, u);

                extended_x.segment<controller::model::number_of_linearized_states>(0)
                    = x0.segment<controller::model::number_of_linearized_states>(0);

                scalar q0 = x0(observer::model::state::quaternion_part_real);
                extended_x.segment<3>(observer::model::state::velocities) -= (Quaternion<scalar>(q0, 0, 0, -std::sqrt(1-SQR(q0)))).toRotationMatrix() * r.segment<3>(0);
                extended_x(observer::model::state::omega_part[Z]) -= r(3);
                regulator.schedule(lmodel);

                u += regulator(extended_x); // Delta u


                #ifdef CRAP_OUTPUT_DATA
                    Aout << lmodel.A.format(IOFormat(FullPrecision)) << std::endl;
                    Bout << lmodel.B.format(IOFormat(FullPrecision)) << std::endl;
                    Qout << lmodel.Q.format(IOFormat(FullPrecision)) << std::endl;
                    Rout << lmodel.R.format(IOFormat(FullPrecision)) << std::endl;
                    rout << r.transpose().format(IOFormat(FullPrecision)) << std::endl;
                    Lout << regulator.L.format(IOFormat(FullPrecision)) << std::endl;
                    Lrout << regulator.Lr_inv.solve(Matrix<scalar, 4, 4>::Identity()).format(IOFormat(FullPrecision)) << std::endl;
                    eXout << extended_x.transpose().format(IOFormat(FullPrecision)) << std::endl;
                    uout << regulator(extended_x).transpose().format(IOFormat(FullPrecision)) << std::endl;
                #endif

                #ifdef CRAP_PLOT
                    u0plot << std::make_pair(CRAP::starting_time.elapsed(), u(0));
                    u1plot << std::make_pair(CRAP::starting_time.elapsed(), u(1));
                    u2plot << std::make_pair(CRAP::starting_time.elapsed(), u(2));
                    u3plot << std::make_pair(CRAP::starting_time.elapsed(), u(3));
                #endif
            }

            void init_controller() {
                using namespace observer::model;

                r.setZero();

                lmodel.A(controller::model::state::roll, state::omega_part[X]) = 1;
                lmodel.A(controller::model::state::pitch, state::omega_part[Y]) = 1;

                lmodel.Q(controller::model::number_of_states,controller::model::number_of_states) = 0;
                lmodel.Q.block<4,4>(observer::model::state::rotor_velocities,state::rotor_velocities).setZero();

                lmodel.Q(observer::model::state::omega_part[Z], state::omega_part[X]) = config["state_weights"]["omega_x"].as<scalar>(1);
                lmodel.Q(observer::model::state::omega_part[Z], state::omega_part[Y]) = config["state_weights"]["omega_y"].as<scalar>(1);
                lmodel.Q(observer::model::state::omega_part[Z], state::omega_part[Z]) = config["state_weights"]["omega_z"].as<scalar>(1e2);
                lmodel.Q(controller::model::state::roll, controller::model::state::roll) = config["state_weights"]["roll"].as<scalar>(1e3);
                lmodel.Q(controller::model::state::pitch, controller::model::state::pitch) = config["state_weights"]["pitch"].as<scalar>(1e3);

                lmodel.R.diagonal().setConstant(config["control_weight"].as<scalar>(1e-2));

                lmodel.A(controller::model::number_of_states, controller::model::number_of_states) = -1e-8;
                extended_x[controller::model::number_of_states] = 1;
                lmodel.M.block<3, 3>(0,state::velocities).setIdentity();
                lmodel.M(3, state::omega_part[Z]) = 1;
                std::cout << "::::: M ::::::" << std::endl << lmodel.M << std::endl << "::::::::::::::::::" << std::endl;
                static const scalar a = config["initial_rotorspeed"].as<scalar>(400.0);
                u << -a, a, -a, a;
            }
        }

        void set_control_reference(const reference_vector& ref) {
            detail::r = ref;
        }

        void receive_control_signal(const observer::model::state_vector& x0) {
            detail::calculate_control_signal(x0);
        }
    }
}

// Module interface
extern "C" {
    using namespace CRAP;
    using namespace CRAP::controller;

    control_vector control_signal() {
        return detail::u;
    }
}

// CRAP interface
extern "C" {
    using namespace CRAP;
    void configure(YAML::Node& c) {
        config = c;
    }

    void run() {
        CRAP::controller::detail::init_controller();
        comm::listen("/state_estimate", controller::receive_control_signal);
        comm::listen(config["channels"]["reference"].as<std::string>("/reference"), controller::set_control_reference);
    }
}
