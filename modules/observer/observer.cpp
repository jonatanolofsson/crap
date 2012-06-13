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
#include "math/math.hpp"
#include "math/filtering/filtering.hpp"
#include "math/filtering/ekf.hpp"
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <fstream>

#include "modules/sensor_reader/sensor_data.hpp"

namespace CRAP {
    namespace observer {
        #ifdef CRAP_PLOT
            using namespace cpplot;

            template<int rows, int cols>
            int index(int row, int col) { return row + (col-1)*rows; }

            static const int count = 30*10;
            #define rows 3
            #define cols 2

            auto PRplot = figure("Observer")->subplot(rows,cols,index<rows,cols>(1,2))->title("Roll")->add<Line>()->set("r")->set_capacity(count);
            auto PPplot = figure("Observer")->subplot(rows,cols,index<rows,cols>(2,2))->title("Pitch")->add<Line>()->set("r")->set_capacity(count);
            auto PYplot = figure("Observer")->subplot(rows,cols,index<rows,cols>(3,2))->title("Yaw")->add<Line>()->set("r")->set_capacity(count);
            auto oPRplot = figure("Observer")->subplot(rows,cols,index<rows,cols>(1,2))->add<Line>()->set("b")->set_capacity(count);
            auto oPPplot = figure("Observer")->subplot(rows,cols,index<rows,cols>(2,2))->add<Line>()->set("b")->set_capacity(count);
            auto oPYplot = figure("Observer")->subplot(rows,cols,index<rows,cols>(3,2))->add<Line>()->set("b")->set_capacity(count);

            auto Pxplot = figure("Observer")->subplot(rows,cols,index<rows,cols>(1,1))->title("Position X")->add<Line>()->set("r")->set_capacity(count);
            auto Pyplot = figure("Observer")->subplot(rows,cols,index<rows,cols>(2,1))->title("Position Y")->add<Line>()->set("r")->set_capacity(count);
            auto Pzplot = figure("Observer")->subplot(rows,cols,index<rows,cols>(3,1))->title("Position Z")->add<Line>()->set("r")->set_capacity(count);
            auto oPxplot = figure("Observer")->subplot(rows,cols,index<rows,cols>(1,1))->add<Line>()->set("b")->set_capacity(count);
            auto oPyplot = figure("Observer")->subplot(rows,cols,index<rows,cols>(2,1))->add<Line>()->set("b")->set_capacity(count);
            auto oPzplot = figure("Observer")->subplot(rows,cols,index<rows,cols>(3,1))->add<Line>()->set("b")->set_capacity(count);

            #undef rows
            #undef cols
        #endif

        void init(bool = false);

        typedef filtering::KalmanFilter<model::number_of_states, model::number_of_controls> KF;
        typedef base_float_t scalar;
        using namespace filtering;
        using namespace model;
        boost::condition_variable trigger;
        bool has_started = false;

        YAML::Node config;
        bool use_control_signal = false;

        controller::control_signal_fn ctrl_signal;
        control_vector u;
        KF filter; ///< Filter instance
        #ifdef CRAP_OUTPUT_DATA
            namespace output {
                //~ log::filelogger observer("observer.log",{"time", "velX", "velY", "velZ", "wRoll", "wPitch", "wYaw", "wr1", "wr2", "wr3", "wr4", "qwbi", "qwbj", "qwbk", "qwb0", "X", "Y", "Z", "windX", "windY", "windZ", "driftRoll", "driftPitch", "driftYaw", "camScale","abiasX","abiasY","abiasZ"});
                log::filelogger observer("observer.log",{"time", "velX", "velY", "velZ", "wRoll", "wPitch", "wYaw", "wr1", "wr2", "wr3", "wr4", "qwbi", "qwbj", "qwbk", "qwb0", "X", "Y", "Z", "driftRoll", "driftPitch", "driftYaw", "windX", "windY", "windZ"});
                log::filelogger covariance("covariance.log");
                log::filelogger camera("camera_measured.log", {"time", "qPTAM0", "qPTAMi", "qPTAMj", "qPTAMk", "XPTAM", "YPTAM", "ZPTAM"});
                log::filelogger camera_sim("camera_simulated.log", {"time", "qPTAM0", "qPTAMi", "qPTAMj", "qPTAMk", "XPTAM", "YPTAM", "ZPTAM"});
                log::filelogger imu("imu_measured.log", {"time", "accX", "accY", "accZ", "wRoll", "wPitch", "wYaw"});
                log::filelogger imu_sim("imu_simulated.log", {"time", "accX", "accY", "accZ", "wRoll", "wPitch", "wYaw"});
            }
        #endif

        namespace predict {
            typedef prediction_model<model::number_of_states, model::number_of_controls> prediction_model_t;
            prediction_model_t prediction(model::f);
        }

        struct kf_collect {
            KF::state_vector x;
            KF::state_vector P;
            kf_collect(KF::state_vector x_, KF::state_vector P_) : x(x_), P(P_){};
        };

        void publish() {
            //~ std::cout << "Publish" << std::endl;
            boost::mutex::scoped_lock l(filter.lock);
            ::CRAP::comm::broadcast("/control_model", filtering::control_model<model::number_of_states, model::number_of_controls>(predict::prediction.F, filter.x));
            ::CRAP::comm::send("/state_estimate", filter.x);
            ::CRAP::comm::send("/state_estimate_brief", kf_collect(filter.x, filter.P.diagonal()));
            #ifdef CRAP_OUTPUT_DATA
                double t = CRAP::starting_time.elapsed();
                output::observer(t,filter.x);
                output::covariance(filter.P);
            #endif
            //~ std::cout << "Sending state: " << filter.x.transpose() << std::endl;
        }

        namespace observe {
            using namespace Eigen;
            namespace imu_sensor {
                typedef observation_model<model::number_of_states, model::imu::data_size, base_float_t> observation_t;
                observation_t imu_model(model::imu::measurement);

                void initialize(const observation_t::measurement_vector& z) {
                    //~ boost::mutex::scoped_lock l(filter.lock);
                    //~ filter.x.segment<3>(state::gyro_offset) = z.segment<3>(imu::gyroscope);
                }

                void measurement(const observation<model::imu::data_size>& obs) {
                    if(!has_started) return;
                    boost::mutex::scoped_lock l(filter.lock);
                    static bool first = true;
                    if(first) {
                        initialize(obs.z);
                        first = false;
                    }
                    #if defined(CRAP_OUTPUT_DATA)
                        double t = CRAP::starting_time.elapsed();
                        observation_t::measurement_vector meas(model::imu::measurement(filter.x));
                    #endif
                    #ifdef CRAP_OUTPUT_DATA
                        output::imu(t, obs.z);
                        output::imu_sim(t, meas);
                    #endif
                    //~ std::cout << "IMU measurement: :::::::::::::" << std::endl;
                    //~ std::cout << "Before: " << filter.x.transpose() << std::endl;
                    ekf::observe(filter, imu_model, obs);
                    filter.x.segment<4>(state::quaternion).normalize();
                    //~ std::cout << "Got: " << obs.z.transpose() << std::endl;
                    //~ std::cout << "Sim: " << meas.transpose() << std::endl;
                    //~ std::cout << "After: " << filter.x.transpose() << std::endl;
                    //~ filter.x.segment<4>(state::qPw).normalize();
                }
            }

            namespace camera_sensor {
                typedef observation_model<model::number_of_states, model::camera::data_size, base_float_t> observation_t;
                observation_t camera_model(model::camera::measurement);
                Matrix<scalar, 3, 1> last_XPTAM;
                scalar teleportation_eps;

                //~ void initialize_covariance(state_covariance_matrix& P) {
                    //~ P(state::camera_scale, state::camera_scale) = config["P0"]["camera_scale"].as<state::scalar>(1e-1);
                    //~ P.block<3,3>(state::camera_origin, state::camera_origin) = Matrix<scalar, 3, 3>::Identity() * config["P0"]["camera_origin"].as<state::scalar>(1e-1);
                    //~ P.block<4,4>(state::qPw, state::qPw) = Matrix<scalar, 4, 4>::Identity() * config["P0"]["qPw"].as<state::scalar>(1e-1);
                //~ }

                void initialize(const observation_t::measurement_vector& z) {
                    std::cout << "Initializing observer camera states" << std::endl;
                    //~ boost::mutex::scoped_lock l(filter.lock);
                    Matrix<scalar, 3, 1> XPTAM = z.segment<3>(camera::position);
                    Quaternion<scalar> qPTAM(
                        z(camera::quaternion_part_real),
                        z(camera::quaternion_part_vector[0]),
                        z(camera::quaternion_part_vector[1]),
                        z(camera::quaternion_part_vector[2])
                    );

                    Quaternion<double> qbc(
                        AngleAxis<double>(config["camera"]["tilt"].as<double>(), Vector3d::UnitY())
                        * AngleAxis<double>(config["camera"]["yaw"].as<double>(), Vector3d::UnitZ())
                        * AngleAxis<double>(M_PI_2, Vector3d::UnitZ())
                        * AngleAxis<double>(M_PI_2, Vector3d::UnitX())
                    );

                    CRAP::model::camera::initialize(filter.x, XPTAM, qPTAM, qbc);
                    //~ initialize_covariance(filter.P);
                    std::cout << "Finished camera initialization" << std::endl;
                    trigger.notify_all();
                    //~ CRAP::starting_time.restart();
                }

                void measurement(const observation<camera::data_size>& obs) {
                    //~ std::cout << "Received camera measurement: " <<  obs.z.transpose() << std::endl;
                    boost::mutex::scoped_lock l(filter.lock);
                    //~ const observation<camera::data_size> obs(obs_.z, obs_.R * filter.x(state::camera_scale));
                    //~ std::cout << "Camera got lock" << std::endl;
                    Matrix<scalar, 3, 1> XPTAM = obs.z.segment<3>(camera::position);
                    static bool first = true;
                    if(first) {// || BIT_TST(obs.flags, camera::REINITIALIZE_BIT) || (XPTAM - last_XPTAM).norm()*filter.x(state::camera_scale) > teleportation_eps) {
                        observer::init(true);
                        //~ if(first) {
                            filter.x.segment<3>(state::position) << config["camera"]["initial_state"]["position"][0].as<scalar>(),config["camera"]["initial_state"]["position"][1].as<scalar>(),config["camera"]["initial_state"]["position"][2].as<scalar>();
                            filter.x.segment<4>(state::quaternion) << eul2quat(config["camera"]["initial_state"]["orientation"][0].as<scalar>(0.0),config["camera"]["initial_state"]["orientation"][1].as<scalar>(0.0),config["camera"]["initial_state"]["orientation"][2].as<scalar>(0.0));
                        //~ }
                        initialize(obs.z);//, first || BIT_TST(obs.flags, camera::REINITIALIZE_SCALE_BIT));
                        first = false;
                    }
                    last_XPTAM = XPTAM;

                    #if defined(CRAP_PLOT) || defined(CRAP_OUTPUT_DATA)
                        double t = CRAP::starting_time.elapsed();
                        observation_t::measurement_vector meas(model::camera::measurement(filter.x));
                    #endif

                    #ifdef CRAP_PLOT
                        Vector3d oeul; oeul << quat2eul(obs.z(camera::quaternion_part_real),obs.z(camera::quaternion_part_vector[0]),obs.z(camera::quaternion_part_vector[1]),obs.z(camera::quaternion_part_vector[2]));
                        Vector3d eul; eul << quat2eul(meas(camera::quaternion_part_real),meas(camera::quaternion_part_vector[0]),meas(camera::quaternion_part_vector[1]),meas(camera::quaternion_part_vector[2]));
                        oPRplot << std::make_pair(t, oeul[0]);
                        oPPplot << std::make_pair(t, oeul[1]);
                        oPYplot << std::make_pair(t, oeul[2]);
                        PRplot << std::make_pair(t, eul[0]);
                        PPplot << std::make_pair(t, eul[1]);
                        PYplot << std::make_pair(t, eul[2]);

                        oPxplot << std::make_pair(t, XPTAM(0));
                        oPyplot << std::make_pair(t, XPTAM(1));
                        oPzplot << std::make_pair(t, XPTAM(2));

                        Pxplot << std::make_pair(t, meas(camera::positions[X]));
                        Pyplot << std::make_pair(t, meas(camera::positions[Y]));
                        Pzplot << std::make_pair(t, meas(camera::positions[Z]));
                    #endif

                    #ifdef CRAP_OUTPUT_DATA
                        output::camera(t, obs.z);
                        output::camera_sim(t, meas);
                        //~ output::camera_sim(t, (Matrix<double, model::camera::data_size+1, 1>() << meas, filter.x(state::camera_scale)).finished());
                    #endif
                    ekf::observe(filter, camera_model, obs);
                    filter.x.segment<4>(state::quaternion).normalize();
                    //~ filter.x.segment<4>(state::qPw).normalize();
                }
            }
        }

        namespace predict {
            void time_update() {
                boost::mutex::scoped_lock l(filter.lock);
                //~ std::cout << ":::::::::::::::::::::::::::::" << std::endl;
                if(use_control_signal) {
                    u = ctrl_signal();
                    //~ std::cout << "Received control signal: " << u.transpose() << std::endl;
                }
                //~ filter.x = model::f(filter.x, u);
                ekf::predict<model::number_of_states, model::number_of_controls>(filter, prediction, u);
                filter.x.segment<4>(state::quaternion).normalize();
                //~ filter.x.segment<4>(state::qPw).normalize();
                //~ std::cout << filter.x.segment<3>(state::position).transpose() << std::endl;
                //~ std::cout << ":::::::::::::::::::::::::::::" << std::endl;
            }
            void time_update2() {
                boost::mutex::scoped_lock l(filter.lock);
                filter.P += prediction.Q;
                filter.x.segment<4>(state::quaternion).normalize();
            }
        }

        void init(bool got_lock) {
            if(!got_lock) boost::mutex::scoped_lock l(filter.lock);
            filter.x.setZero();
            filter.x.segment<3>(state::position) <<
                config["initial_state"]["position"][0].as<state::scalar>(0.0),
                config["initial_state"]["position"][1].as<state::scalar>(0.0),
                config["initial_state"]["position"][2].as<state::scalar>(0.0);
            filter.x.segment<4>(state::quaternion) << eul2quat(config["initial_state"]["orientation"][0].as<state::scalar>(0.0), config["initial_state"]["orientation"][1].as<state::scalar>(0.0), config["initial_state"]["orientation"][2].as<state::scalar>(0.0));
            //~ std::cout << "Initial quaternion: " << filter.x.segment<4>(state::quaternion).transpose() << std::endl;

            //~ filter.x.segment<4>(state::qPw) << eul2quat(0, 0, 0);
            //~ filter.x(state::camera_scale)  = 1.0;

            filter.x.segment<4>(state::rotor_velocities) <<
                -config["initial_state"]["rotor_velocities"].as<state::scalar>(0.0), // Forward
                config["initial_state"]["rotor_velocities"].as<state::scalar>(0.0), // Left
                -config["initial_state"]["rotor_velocities"].as<state::scalar>(0.0), // Back
                config["initial_state"]["rotor_velocities"].as<state::scalar>(0.0); // Right


            //~ if(use_control_signal) u = ctrl_signal();
            //~ else
            u.setConstant(config["initial_state"]["rotor_velocities"].as<state::scalar>(0.0));

            //~ filter.x.segment<3>(state::accelerometer_bias) <<
                //~ config["initial_state"]["accelerometer_bias"][0].as<state::scalar>(0.0),
                //~ config["initial_state"]["accelerometer_bias"][1].as<state::scalar>(0.0),
                //~ config["initial_state"]["accelerometer_bias"][2].as<state::scalar>(0.0);

            state_covariance_matrix P0;
            P0.setZero();
            P0.diagonal()(state::velocities[X]) = config["P0"]["velocity"][0].as<state::scalar>(3e-1);
            P0.diagonal()(state::velocities[Y]) = config["P0"]["velocity"][0].as<state::scalar>(3e-1);
            P0.diagonal()(state::velocities[Z]) = config["P0"]["velocity"][1].as<state::scalar>(3e-1);

            P0.diagonal()(state::omega[X]) = config["P0"]["omega"][0].as<state::scalar>(3e-2);
            P0.diagonal()(state::omega[Y]) = config["P0"]["omega"][0].as<state::scalar>(3e-2);
            P0.diagonal()(state::omega[Z]) = config["P0"]["omega"][1].as<state::scalar>(3e-2);

            P0.diagonal()(state::omega_r[0]) = config["P0"]["omega_r"].as<state::scalar>(1e1);
            P0.diagonal()(state::omega_r[1]) = config["P0"]["omega_r"].as<state::scalar>(1e1);
            P0.diagonal()(state::omega_r[2]) = config["P0"]["omega_r"].as<state::scalar>(1e1);
            P0.diagonal()(state::omega_r[3]) = config["P0"]["omega_r"].as<state::scalar>(1e1);

            P0.diagonal()(state::quaternion_part_real) = config["P0"]["quaternion"][0].as<state::scalar>(3e-2);
            P0.diagonal()(state::quaternion_part_vector[0]) = config["P0"]["quaternion"][1].as<state::scalar>(3e-2);
            P0.diagonal()(state::quaternion_part_vector[1]) = config["P0"]["quaternion"][1].as<state::scalar>(3e-2);
            P0.diagonal()(state::quaternion_part_vector[2]) = config["P0"]["quaternion"][2].as<state::scalar>(3e-2);

            P0.diagonal()(state::positions[X]) = config["P0"]["position"][0].as<state::scalar>(1e-1);
            P0.diagonal()(state::positions[Y]) = config["P0"]["position"][0].as<state::scalar>(1e-1);
            P0.diagonal()(state::positions[Z]) = config["P0"]["position"][1].as<state::scalar>(1e-1);

            P0.diagonal()(state::wind_velocities[X]) = config["P0"]["wind"][0].as<state::scalar>(1e-1);
            P0.diagonal()(state::wind_velocities[Y]) = config["P0"]["wind"][0].as<state::scalar>(1e-1);
            P0.diagonal()(state::wind_velocities[Z]) = config["P0"]["wind"][1].as<state::scalar>(1e-1);

            P0.diagonal()(state::gyro_offsets[X]) = config["P0"]["gyro_offset"].as<state::scalar>(6e-3);
            P0.diagonal()(state::gyro_offsets[Y]) = config["P0"]["gyro_offset"].as<state::scalar>(6e-3);
            P0.diagonal()(state::gyro_offsets[Z]) = config["P0"]["gyro_offset"].as<state::scalar>(6e-3);

            //~ P0.diagonal()(state::qPw_real) = config["P0"]["qPw"].as<state::scalar>(3e-2);
            //~ P0.diagonal()(state::qPw_vector[0]) = config["P0"]["qPw"].as<state::scalar>(3e-2);
            //~ P0.diagonal()(state::qPw_vector[1]) = config["P0"]["qPw"].as<state::scalar>(3e-2);
            //~ P0.diagonal()(state::qPw_vector[2]) = config["P0"]["qPw"].as<state::scalar>(3e-2);
//~
            //~ P0.diagonal()(state::camera_origins[X]) = config["P0"]["camera_origin"].as<state::scalar>(1e-1);
            //~ P0.diagonal()(state::camera_origins[Y]) = config["P0"]["camera_origin"].as<state::scalar>(1e-1);
            //~ P0.diagonal()(state::camera_origins[Z]) = config["P0"]["camera_origin"].as<state::scalar>(1e-1);

            //~ P0.diagonal()(state::camera_scale) = config["P0"]["camera_scale"].as<state::scalar>(1e-1);
//~
            //~ P0.diagonal()(state::accelerometer_biases[X]) = config["P0"]["accelerometer_bias"][0].as<state::scalar>(1e-4);
            //~ P0.diagonal()(state::accelerometer_biases[Y]) = config["P0"]["accelerometer_bias"][1].as<state::scalar>(1e-4);
            //~ P0.diagonal()(state::accelerometer_biases[Z]) = config["P0"]["accelerometer_bias"][2].as<state::scalar>(1e-4);
            filter.P = P0;
            //~ std::cout << "Initial P: " << std::endl << filter.P << std::endl;


            state_covariance_matrix Q_; ///< Internal storage for state covariance
            Q_.setZero();
            Q_.diagonal()(state::velocities[X]) = config["Q"]["velocity"][0].as<state::scalar>(3e-2);
            Q_.diagonal()(state::velocities[Y]) = config["Q"]["velocity"][0].as<state::scalar>(3e-2);
            Q_.diagonal()(state::velocities[Z]) = config["Q"]["velocity"][1].as<state::scalar>(3e-2);

            Q_.diagonal()(state::omega[X]) = config["Q"]["omega"][0].as<state::scalar>(3e-3);
            Q_.diagonal()(state::omega[Y]) = config["Q"]["omega"][0].as<state::scalar>(3e-3);
            Q_.diagonal()(state::omega[Z]) = config["Q"]["omega"][1].as<state::scalar>(3e-3);

            Q_.diagonal()(state::omega_r[0]) = config["Q"]["omega_r"].as<state::scalar>(5e0);
            Q_.diagonal()(state::omega_r[1]) = config["Q"]["omega_r"].as<state::scalar>(5e0);
            Q_.diagonal()(state::omega_r[2]) = config["Q"]["omega_r"].as<state::scalar>(5e0);
            Q_.diagonal()(state::omega_r[3]) = config["Q"]["omega_r"].as<state::scalar>(5e0);

            Q_.diagonal()(state::quaternion_part_real) = config["Q"]["quaternion"][0].as<state::scalar>(3e-3);
            Q_.diagonal()(state::quaternion_part_vector[0]) = config["Q"]["quaternion"][1].as<state::scalar>(3e-3);
            Q_.diagonal()(state::quaternion_part_vector[1]) = config["Q"]["quaternion"][1].as<state::scalar>(3e-3);
            Q_.diagonal()(state::quaternion_part_vector[2]) = config["Q"]["quaternion"][2].as<state::scalar>(3e-3);

            Q_.diagonal()(state::positions[X]) = config["Q"]["position"][0].as<state::scalar>(3e-5);
            Q_.diagonal()(state::positions[Y]) = config["Q"]["position"][0].as<state::scalar>(3e-5);
            Q_.diagonal()(state::positions[Z]) = config["Q"]["position"][1].as<state::scalar>(3e-5);

            Q_.diagonal()(state::wind_velocities[X]) = config["Q"]["wind"].as<state::scalar>(3e-1);
            Q_.diagonal()(state::wind_velocities[Y]) = config["Q"]["wind"].as<state::scalar>(3e-1);
            Q_.diagonal()(state::wind_velocities[Z]) = config["Q"]["wind"].as<state::scalar>(3e-1);

            Q_.diagonal()(state::gyro_offsets[X]) = config["Q"]["gyro_offset"].as<state::scalar>(3e-4);
            Q_.diagonal()(state::gyro_offsets[Y]) = config["Q"]["gyro_offset"].as<state::scalar>(3e-4);
            Q_.diagonal()(state::gyro_offsets[Z]) = config["Q"]["gyro_offset"].as<state::scalar>(3e-4);

            //~ Q_.diagonal()(state::qPw_real) = config["Q"]["qPw"].as<state::scalar>(3e-2);
            //~ Q_.diagonal()(state::qPw_vector[0]) = config["Q"]["qPw"].as<state::scalar>(3e-2);
            //~ Q_.diagonal()(state::qPw_vector[1]) = config["Q"]["qPw"].as<state::scalar>(3e-2);
            //~ Q_.diagonal()(state::qPw_vector[2]) = config["Q"]["qPw"].as<state::scalar>(3e-2);
//~
            //~ Q_.diagonal()(state::camera_origins[X]) = config["Q"]["camera_origin"].as<state::scalar>(1e-1);
            //~ Q_.diagonal()(state::camera_origins[Y]) = config["Q"]["camera_origin"].as<state::scalar>(1e-1);
            //~ Q_.diagonal()(state::camera_origins[Z]) = config["Q"]["camera_origin"].as<state::scalar>(1e-1);

            //~ Q_.diagonal()(state::camera_scale) = config["Q"]["camera_scale"].as<state::scalar>(1e-1);
//~
            //~ Q_.diagonal()(state::accelerometer_biases[X]) = config["Q"]["accelerometer_bias"][0].as<state::scalar>(1e-4);
            //~ Q_.diagonal()(state::accelerometer_biases[Y]) = config["Q"]["accelerometer_bias"][1].as<state::scalar>(1e-4);
            //~ Q_.diagonal()(state::accelerometer_biases[Z]) = config["Q"]["accelerometer_bias"][2].as<state::scalar>(1e-4);

            predict::prediction.Q = Q_ * model::T;


            std::cout << "Observer starting state: " << filter.x.transpose() << std::endl;
        }
    }
}

extern "C" {
    using namespace CRAP::observer;
    state_vector get_state() {
        boost::mutex::scoped_lock l(filter.lock);
        return filter.x;
    }
    state_covariance_matrix get_covariance() {
        boost::mutex::scoped_lock l(filter.lock);
        return filter.P;
    }
}

extern "C" {
    using namespace CRAP;
    using namespace CRAP::observer;
    void configure(YAML::Node& c) {
        std::cout << "Reconfiguring observer: " << std::endl;
        config = c;
        observe::camera_sensor::teleportation_eps = c["teleportation_eps"].as<scalar>();
        observer::init();
    }

    bool running = true;
    time::frequency_t frequency(model::frequency);

    void run() {
        std::cout << "Running the observer" << std::endl;
        if(config["use_control_signal"].as<bool>(true)) {
            std::cout << "Using control signal" << std::endl;
            model::enable_controller();
            observer::ctrl_signal = comm::bind<controller::control_signal_fn>("controller", "control_signal");
            observer::use_control_signal = true;
        }

        comm::listen(config["topics"]["imu"].as<std::string>("/imu"), observe::imu_sensor::measurement);
        comm::listen(config["topics"]["pose"].as<std::string>("/pose"), observe::camera_sensor::measurement);

        if(config["wait_for_trigger"].as<bool>()) {
            boost::mutex lock;
            boost::unique_lock<boost::mutex> l(lock);
            trigger.wait(l);
            std::cout << "Finished waiting" << std::endl;
        }
        has_started = true;

        if(config["use_time_update"].as<bool>(true)) {
            while(running && time::ticktock(::frequency)) {
                observer::predict::time_update();
                publish();
            }
        } else {
            while(running && time::ticktock(::frequency)) {
                observer::predict::time_update2();
                publish();
            }
        }
    }

    void stop() {
        running = false;
    }
}
