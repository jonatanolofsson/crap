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
#include "math/filtering/filtering.hpp"
#include "math/control/LQ.hpp"
#include "math/math.hpp"
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/thread/mutex.hpp>

namespace CRAP {
    namespace controller {
        typedef filtering::control_model<CRAP::model::number_of_states, CRAP::model::number_of_controls> control_model_t;
        YAML::Node config;

        #ifdef CRAP_OUTPUT_DATA
            namespace output {
                log::filelogger reference("reference.log", {"time", "velX", "velY", "velZ", "wYaw"});
                log::filelogger controlstates("controlstate.log", {"time", "velX", "velY", "velZ", "wYaw"});
                log::filelogger control("controlsignal.log", {"time", "wr1", "wr2", "wr3", "wr4"});
            }
            //~ std::ofstream Aout("A.mat");
            //~ std::ofstream Bout("B.mat");
            //~ std::ofstream Qout("Q.mat");
            //~ std::ofstream Rout("R.mat");
            //~ std::ofstream rout("r.mat");
            //~ std::ofstream Lout("L.mat");
            //~ std::ofstream Lrout("Lr.mat");
            //~ std::ofstream eXout("eX.mat");
            //~ std::ofstream uout("u.mat");
        #endif

        #ifdef CRAP_PLOT
            using namespace cpplot;
            static const int count = 30*10;
            auto u0plot = figure("Control signal")->subplot(2,1,1)->title("u")->add<Line>()->set_capacity(count)->set("r");
            auto u1plot = figure("Control signal")->subplot(2,1,2)->add<Line>()->set_capacity(count)->set("g");
            auto u2plot = figure("Control signal")->subplot(2,1,1)->add<Line>()->set_capacity(count)->set("b");
            auto u3plot = figure("Control signal")->subplot(2,1,2)->add<Line>()->set_capacity(count)->set("k");

            auto e0plot = figure("Control errors")->subplot(1,1,1)->title("e")->add<Line>()->set_capacity(count)->set("r");
            auto e1plot = figure("Control errors")->subplot(1,1,1)->add<Line>()->set_capacity(count)->set("g");
            auto e2plot = figure("Control errors")->subplot(1,1,1)->add<Line>()->set_capacity(count)->set("b");
            auto e3plot = figure("Control errors")->subplot(1,1,1)->add<Line>()->set_capacity(count)->set("k");

            auto x0plot = figure("Control variables")->subplot(4,1,1)->title("Vx")->add<Line>()->set_capacity(count)->set("b");
            auto x1plot = figure("Control variables")->subplot(4,1,2)->title("Vy")->add<Line>()->set_capacity(count)->set("b");
            auto x2plot = figure("Control variables")->subplot(4,1,3)->title("Vz")->add<Line>()->set_capacity(count)->set("b");
            auto x3plot = figure("Control variables")->subplot(4,1,4)->title("Wz")->add<Line>()->set_capacity(count)->set("b");

            auto r0plot = figure("Control variables")->subplot(4,1,1)->add<Line>()->set_capacity(count)->set("r");
            auto r1plot = figure("Control variables")->subplot(4,1,2)->add<Line>()->set_capacity(count)->set("r");
            auto r2plot = figure("Control variables")->subplot(4,1,3)->add<Line>()->set_capacity(count)->set("r");
            auto r3plot = figure("Control variables")->subplot(4,1,4)->add<Line>()->set_capacity(count)->set("r");
        #endif


        namespace detail {
            boost::mutex u_lock;
            control_vector u;
            reference_vector r;

            typedef Matrix<scalar, CRAP::model::number_of_states, 1> full_state_vector;
            typedef CRAP::control::LQ<model::number_of_states+1, model::number_of_controls, model::number_of_references> controller_type;
            typedef controller_type::controller_model lmodel_t;
            lmodel_t lmodel;
            controller_type::state_vector extended_x;
            controller_type regulator;


            void linearize(const control_model_t& ctrl_model) {
                lmodel.A.block<controller::model::number_of_states, controller::model::number_of_states>(0,0)
                    = ctrl_model.F.block<controller::model::number_of_states, controller::model::number_of_states>(0,0);

                lmodel.A.block<controller::model::number_of_states, 1>(0, controller::model::number_of_states)
                    = CRAP::model::f(ctrl_model.x, u).segment<controller::model::number_of_states>(0)
                        - lmodel.A.block<controller::model::number_of_states, controller::model::number_of_states>(0,0)
                            *ctrl_model.x.segment<controller::model::number_of_states>(0);

                static bool do_B = true;
                if(do_B) {
                    control::control_jacobian<controller::model::number_of_states, controller::model::number_of_controls>(CRAP::model::f, ctrl_model.x, u, lmodel.B);
                    do_B = false;
                }

                //~ std::cout << "A: \n" << lmodel.A << std::endl << std::endl;
            }

            void calculate_control_signal(const control_model_t& ctrl_model) {
                using CRAP::model::Z;
                boost::mutex::scoped_lock l(u_lock);
                linearize(ctrl_model);

                extended_x.segment<controller::model::number_of_states>(0)
                    = ctrl_model.x.segment<controller::model::number_of_states>(0);

                Quaternion<scalar> q(
                    ctrl_model.x(CRAP::model::state::quaternion_part_real),
                    ctrl_model.x(CRAP::model::state::quaternion_part_vector[0]),
                    ctrl_model.x(CRAP::model::state::quaternion_part_vector[1]),
                    ctrl_model.x(CRAP::model::state::quaternion_part_vector[2])
                );
                extended_x.segment<3>(CRAP::model::state::velocity) -= r.segment<3>(0);
                extended_x(CRAP::model::state::omega[Z]) -= r(3);
                regulator.schedule(lmodel);

                u += regulator(extended_x); // Delta u

                #if defined(CRAP_OUTPUT_DATA) || defined(CRAP_PLOT)
                    double t = CRAP::starting_time.elapsed();
                #endif

                #ifdef CRAP_OUTPUT_DATA
                    output::reference(t, r);
                    output::controlstates(t, (Vector4d() <<
                        ctrl_model.x(CRAP::model::state::velocities[0]),
                        ctrl_model.x(CRAP::model::state::velocities[1]),
                        ctrl_model.x(CRAP::model::state::velocities[2]),
                        ctrl_model.x(CRAP::model::state::omega[2])).finished());
                    output::control(t,u);
                    //~ Aout << lmodel.A.format(IOFormat(FullPrecision)) << std::endl;
                    //~ Bout << lmodel.B.format(IOFormat(FullPrecision)) << std::endl;
                    //~ Qout << lmodel.Q.format(IOFormat(FullPrecision)) << std::endl;
                    //~ Rout << lmodel.R.format(IOFormat(FullPrecision)) << std::endl;
                    //~ rout << r.transpose().format(IOFormat(FullPrecision)) << std::endl;
                    //~ Lout << regulator.L.format(IOFormat(FullPrecision)) << std::endl;
                    //~ ////~ Lrout << regulator.Lr_inv.solve(Matrix<scalar, 4, 4>::Identity()).format(IOFormat(FullPrecision)) << std::endl;
                    //~ eXout << extended_x.transpose().format(IOFormat(FullPrecision)) << std::endl;
                    //~ uout << regulator(extended_x).transpose().format(IOFormat(FullPrecision)) << std::endl;
                #endif

                #ifdef CRAP_PLOT
                    u0plot << std::make_pair(t, u(0));
                    u1plot << std::make_pair(t, u(1));
                    u2plot << std::make_pair(t, u(2));
                    u3plot << std::make_pair(t, u(3));

                    x0plot << std::make_pair(t, ctrl_model.x(CRAP::model::state::velocities[0]));
                    x1plot << std::make_pair(t, ctrl_model.x(CRAP::model::state::velocities[1]));
                    x2plot << std::make_pair(t, ctrl_model.x(CRAP::model::state::velocities[2]));
                    x3plot << std::make_pair(t, ctrl_model.x(CRAP::model::state::omega[2]));

                    r0plot << std::make_pair(t, r(0));
                    r1plot << std::make_pair(t, r(1));
                    r2plot << std::make_pair(t, r(2));
                    r3plot << std::make_pair(t, r(3));

                    e0plot << std::make_pair(t, extended_x(CRAP::model::state::velocities[0]));
                    e1plot << std::make_pair(t, extended_x(CRAP::model::state::velocities[1]));
                    e2plot << std::make_pair(t, extended_x(CRAP::model::state::velocities[2]));
                    e3plot << std::make_pair(t, extended_x(CRAP::model::state::omega[2]));
                #endif
            }

            void init_controller() {
                using namespace CRAP::model;

                r.setZero();

                lmodel.DICO = 'D';

                lmodel.Q(controller::model::number_of_states,controller::model::number_of_states) = 0;
                lmodel.Q.block<4,4>(CRAP::model::state::rotor_velocities,CRAP::model::state::rotor_velocities).setZero();

                lmodel.Q(CRAP::model::state::omega[Z], CRAP::model::state::omega[X]) = config["state_weights"]["omega_x"].as<scalar>(1);
                lmodel.Q(CRAP::model::state::omega[Z], CRAP::model::state::omega[Y]) = config["state_weights"]["omega_y"].as<scalar>(1);
                lmodel.Q(CRAP::model::state::omega[Z], CRAP::model::state::omega[Z]) = config["state_weights"]["omega_z"].as<scalar>(1e2);
                lmodel.Q(controller::model::state::roll, controller::model::state::roll) = config["state_weights"]["roll"].as<scalar>(1e3);
                lmodel.Q(controller::model::state::pitch, controller::model::state::pitch) = config["state_weights"]["pitch"].as<scalar>(1e3);

                lmodel.R.diagonal().setConstant(config["control_weight"].as<scalar>(1e-2));

                lmodel.A(controller::model::number_of_states, controller::model::number_of_states) = 1-1e-5;

                extended_x[controller::model::number_of_states] = 1;
                lmodel.M.block<3, 3>(0,state::velocity).setIdentity();
                lmodel.M(3, CRAP::model::state::omega[Z]) = 1;
                //~ std::cout << "::::: M ::::::" << std::endl << lmodel.M << std::endl << "::::::::::::::::::" << std::endl;
                static const scalar a = config["initial_rotorspeed"].as<scalar>(530.0);
                u << -a, a, -a, a;
            }
        }

        void set_control_reference(const reference_vector& ref) {
            detail::r = ref;
            //~ std::cout << "Got new reference: " << ref.transpose() << std::endl;
        }

        void receive_filter(const control_model_t& filter) {
            detail::calculate_control_signal(filter);
        }
    }
}

// Module interface
extern "C" {
    using namespace CRAP;
    using namespace CRAP::controller;

    control_vector control_signal() {
        using namespace CRAP::controller::detail;
        boost::mutex::scoped_lock l(u_lock);
        return u;
    }
}

// CRAP interface
extern "C" {
    using namespace CRAP;
    void configure(YAML::Node& c) {
        CRAP::model::enable_controller();
        config = c;
        CRAP::controller::detail::init_controller();
    }

    void run() {
        comm::listen("/control_model", controller::receive_filter);
        comm::listen(config["channels"]["reference"].as<std::string>("/reference"), controller::set_control_reference);
    }
}
