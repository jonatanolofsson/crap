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
#include "math/math.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"
#include <string>
#include <utility>

#include <Eigen/Core>
#include "crap/modules/observer/model.hpp"
#include "crap/modules/controller/model.hpp"
#include "math/filtering/ukf.hpp"

namespace CRAP {
    namespace observer {
        using namespace filtering;
        using namespace model;

        controller::control_signal_fn u;
        KF filter; ///< Filter instance

        void publish() {
            ::CRAP::comm::send<state_vector>("/state_estimate", filter.x);
            ::CRAP::comm::send<state_covariance_matrix>("/state_estimate/covariance", filter.P);
            //~ std::cout << "Position: " << filter.x(0) << std::endl;
            #ifdef CRAP_PLOT
                using namespace cpplot;
                figure("Observer")->subplot(3,1,1)->gco<Line>()->set_capacity(30*5) << std::make_pair(CRAP::starting_time.elapsed(), filter.x(0));
                figure("Observer")->subplot(3,1,2)->gco<Line>()->set_capacity(30*5) << std::make_pair(CRAP::starting_time.elapsed(), filter.x(1));
                figure("Observer")->subplot(3,1,3)->gco<Line>()->set_capacity(30*5) << std::make_pair(CRAP::starting_time.elapsed(), u()(0));
            #endif
        }

        namespace observe {
            using namespace Eigen;

            typedef Matrix<double, 1,1> taco_t; taco_t v;
            const taco_t& h_tachometer(const state_vector& x) {
                v(0) = x(state::velocity);
                return v;
            }
            void tachometer(const observation<1>& vel) {
                ukf::observe<model::number_of_states,model::number_of_controls,1>(filter, h_tachometer, vel);
                publish();
            }
        }

        namespace predict {
            state_vector xdot; ///< Internal storage for state calculation

            /*!
             * \brief   Calculate the derivative of the states
             *
             * This function contains the expressions for the mathematical
             * derivatives of each state.
             * \param   x   Current state
             * \param   u   Current control signal
             * \return  The derivative of the states at the point defined by x and u
             */
            state_vector& fdot(const state_vector& x, const control_vector& u) {
                xdot(state::position) = x(state::velocity);
                xdot(state::velocity) = (-x(state::velocity) + u(control::velocity)); // /config.tau_v;
                return xdot;
            }

            /*!
             * \brief   Predict the filter's next time step
             *
             * Using the model, current state and the current control signal,
             * the state at the next time-step is predicted using a RK4 method
             * for integrating the state, given the derivatives of the states.
             * \see     fdot
             * \param   x   Current state
             * \param   u   Current control signal
             * \return  Prediction of state in the next time step
             */
            const state_vector& f(const state_vector& x, const control_vector& u) {
                xdot = math::rk4<model::number_of_states, model::number_of_controls>(fdot, x, u, 1.0/observer::model::frequency);
                return xdot;
            }

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

            void time_update() {
                ukf::predict<model::number_of_states, model::number_of_controls>(filter, prediction_model, u());
                publish();
            }
        }
    }
}

extern "C" {
    using namespace CRAP;
    using namespace CRAP::observer;
    void configure(YAML::Node& c) {
        std::cout << "Reconfiguring observer" << std::endl;
        filter.x << 1,
                    1;
        //~ filter.x << c["initial_position"].as<double>(),
                    //~ c["initial_velocity"].as<double>();
        filter.P.setIdentity();
    }

    bool running = true;
    time::frequency_t frequency(model::frequency);

    void run() {
        comm::listen("tachometer", observe::tachometer);
        observer::u = comm::bind<controller::control_signal_fn>("controller", "control_signal");

        while(running && time::ticktock(::frequency)) {
            observer::predict::time_update();
        }
    }

    void stop() {
        running = false;
    }
}
