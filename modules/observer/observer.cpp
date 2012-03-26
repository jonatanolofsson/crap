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
#include <string>
#include <utility>

#include <Eigen/Core>
#include "yaml-cpp/yaml.h"
#include "modules/model/model.hpp"
#include "modules/controller/model.hpp"
#include "math/filtering/filtering.hpp"


#define eul2quat(phi,theta,psi) (sin(phi/2)*cos(theta/2)*cos(psi/2) - cos(phi/2)*sin(theta/2)*sin(psi/2)),(cos(phi/2)*sin(theta/2)*cos(psi/2) + sin(phi/2)*cos(theta/2)*sin(psi/2)),(cos(phi/2)*cos(theta/2)*sin(psi/2) - sin(phi/2)*sin(theta/2)*cos(psi/2)),(cos(phi/2)*cos(theta/2)*cos(psi/2) + sin(phi/2)*sin(theta/2)*sin(psi/2))
static const CRAP::base_float_t deg = M_PI/180.0;

namespace CRAP {
    namespace observer {
        typedef filtering::KalmanFilter<model::number_of_states, model::number_of_controls> KF;
        using namespace filtering;
        using namespace model;

        YAML::Node config;

        controller::control_signal_fn u;
        KF filter; ///< Filter instance

        void publish() {
            ::CRAP::comm::broadcast("/state_estimate", filter.x);
        }

        namespace observe {
            using namespace Eigen;


            //~ typedef Matrix<base_float_t, 3,10> imu_t; imu_t imu_sim;
            //~ const imu_t& h_imu(const state_vector& x) {
                //~ Quaternion<state::scalar> q(x(state::quaternion_part_real), x(state::quaternion_part_vector[0]), x(state::quaternion_part_vector[1]), x(state::quaternion_part_vector[2]));
                //~ q.normalize();
                //~ return imu_sim;
            //~ }
            //~ void imu_measurement(const observation<imu_size>& obs) {
                //~ ukf::observe<model::number_of_states,model::number_of_controls,>(filter, h_tachometer, obs);
                //~ publish();
            //~ }
        }

        namespace predict {
            state_covariance_matrix Q_; ///< Internal storage for state covariance

            /*!
             * \brief   Get the state prediction covariance
             *
             * Calculate and get the estimated covariance of the covariance of
             * the state prediction.
             * \param   x   Current state
             * \param   u   Current control signal
             * \return  Prediction covariance
             */
            const state_covariance_matrix& Q(const state_vector&, const control_vector&)
            {
                return Q_;
            }

            typedef prediction_model_tmpl<model::number_of_states, model::number_of_controls> prediction_model_t;
            prediction_model_t prediction_model(f,Q);
            #ifdef CRAP_PLOT
                using namespace cpplot;
                static const int count = 30*10;
                auto xplot = figure("Observer")->subplot(5,2,1)->title("Position X")->add<Line>()->set_capacity(count);
                auto yplot = figure("Observer")->subplot(5,2,2)->title("Position Y")->add<Line>()->set_capacity(count);
                auto zplot = figure("Observer")->subplot(5,2,3)->title("Position Z")->add<Line>()->set_capacity(count);

                auto vxplot = figure("Observer")->subplot(5,2,6)->title("Velocity X")->add<Line>()->set_capacity(count);
                auto vyplot = figure("Observer")->subplot(5,2,7)->title("Velocity Y")->add<Line>()->set_capacity(count);
                auto vzplot = figure("Observer")->subplot(5,2,8)->title("Velocity Z")->add<Line>()->set_capacity(count);

                auto wplot1 = figure("Observer")->subplot(5,1,4)->title("omega_i")->add<Line>()->set("r")->set_capacity(count);
                auto wplot2 = figure("Observer")->subplot(5,1,4)->add<Line>()->set("g")->set_capacity(count);
                auto wplot3 = figure("Observer")->subplot(5,1,4)->add<Line>()->set("b")->set_capacity(count);
                auto wplot4 = figure("Observer")->subplot(5,1,4)->add<Line>()->set("k")->set_capacity(count);

                auto qplot0 = figure("Observer")->subplot(5,1,5)->title("q")->add<Line>()->set("r")->set_capacity(count);
                auto qploti = figure("Observer")->subplot(5,1,5)->add<Line>()->set("g")->set_capacity(count);
                auto qplotj = figure("Observer")->subplot(5,1,5)->add<Line>()->set("b")->set_capacity(count);
                auto qplotk = figure("Observer")->subplot(5,1,5)->add<Line>()->set("k")->set_capacity(count);
            #endif

            void time_update() {
                //~ std::cout << ":::::::::::::::::::::::::::::" << std::endl;
                filter.x = model::f(filter.x, u());

                #ifdef CRAP_PLOT
                    using namespace cpplot;
                    xplot << std::make_pair(CRAP::starting_time.elapsed(), filter.x(state::position[X]));
                    yplot << std::make_pair(CRAP::starting_time.elapsed(), filter.x(state::position[Y]));
                    zplot << std::make_pair(CRAP::starting_time.elapsed(), filter.x(state::position[Z]));
//~ //~
                    vxplot << std::make_pair(CRAP::starting_time.elapsed(), filter.x(state::velocity[X]));
                    vyplot << std::make_pair(CRAP::starting_time.elapsed(), filter.x(state::velocity[Y]));
                    vzplot << std::make_pair(CRAP::starting_time.elapsed(), filter.x(state::velocity[Z]));
//~ //~
                    wplot1 << std::make_pair(CRAP::starting_time.elapsed(), filter.x(state::rotor_omega[0]));
                    wplot2 << std::make_pair(CRAP::starting_time.elapsed(), filter.x(state::rotor_omega[1]));
                    wplot3 << std::make_pair(CRAP::starting_time.elapsed(), filter.x(state::rotor_omega[2]));
                    wplot4 << std::make_pair(CRAP::starting_time.elapsed(), filter.x(state::rotor_omega[3]));
                    qplot0 << std::make_pair(CRAP::starting_time.elapsed(), filter.x(state::quaternion_part_real));
                    qploti << std::make_pair(CRAP::starting_time.elapsed(), filter.x(state::quaternion_part_vector[0]));
                    qplotj << std::make_pair(CRAP::starting_time.elapsed(), filter.x(state::quaternion_part_vector[1]));
                    qplotk << std::make_pair(CRAP::starting_time.elapsed(), filter.x(state::quaternion_part_vector[2]));
                #endif
                //~ ukf::predict<model::number_of_states, model::number_of_controls>(filter, prediction_model, u());
                //~ filter.x.segment<4>(state::quaternion).normalize();
                publish();
                //~ std::cout << ":::::::::::::::::::::::::::::" << std::endl;
            }
        }

        void init() {
            filter.x.setZero();
            filter.x(state::position[X]) = config["initial_state"]["X"].as<state::scalar>(0.0);
            filter.x(state::position[Y]) = config["initial_state"]["Y"].as<state::scalar>(0.0);
            filter.x(state::position[Z]) = config["initial_state"]["Z"].as<state::scalar>(0.0);
            filter.x.segment<4>(state::quaternion) << eul2quat(0*deg, 0*deg, 0*deg);
            std::cout << "Initial quaternion: " << filter.x.segment<4>(state::quaternion).transpose() << std::endl;
            filter.x.segment<4>(state::rotor_velocities) <<
                -config["initial_state"]["rotor_velocities"].as<state::scalar>(0.0), // Forward
                config["initial_state"]["rotor_velocities"].as<state::scalar>(0.0), // Left
                -config["initial_state"]["rotor_velocities"].as<state::scalar>(0.0), // Back
                config["initial_state"]["rotor_velocities"].as<state::scalar>(0.0); // Right

            filter.P.setZero();
            filter.P.diagonal().setConstant(0.000011);

            predict::Q_.setZero();
            predict::Q_.diagonal().setConstant(0.0001);
        }
    }
}

extern "C" {
    using namespace CRAP::observer;
    state_vector get_state() {
        return filter.x;
    }
}

extern "C" {
    using namespace CRAP;
    using namespace CRAP::observer;
    void configure(YAML::Node& c) {
        std::cout << "Reconfiguring observer: " << std::endl << c << std::endl;
        config = c;
    }

    bool running = true;
    time::frequency_t frequency(model::frequency);

    void run() {
        observer::init();

        //~ comm::listen("tachometer", observe::tachometer);
        observer::u = comm::bind<controller::control_signal_fn>("controller", "control_signal");

        while(running && time::ticktock(::frequency)) {
            observer::predict::time_update();
        }
    }

    void stop() {
        running = false;
    }
}
