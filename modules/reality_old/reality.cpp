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
#include <string>
#include <utility>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

#include <Eigen/Core>
#include "yaml-cpp/yaml.h"
#include "modules/model/model.hpp"
#include "modules/sensor_reader/sensor_data.hpp"
#include "modules/camera_reader/camera_data.hpp"
#include "modules/controller/model.hpp"
#include "math/filtering/filtering.hpp"
#include "linkquad/serial_communication.hpp"
#include <fstream>


namespace CRAP {
    namespace Reality {
        using namespace model;
        using namespace Eigen;
        typedef base_float_t scalar;

        controller::control_signal_fn ctrl_signal;
        control_vector u;
        state_vector x;

        std::ifstream state_file;
        std::ifstream imu_file;
        std::ifstream camera_file;
        Matrix<scalar, model::imu::data_size, 1> imu_data;
        Matrix<scalar, model::camera::data_size, 1> camera_data;
        #ifdef CRAP_OUTPUT_DATA
            namespace output {
                //~ log::filelogger reality("reality.log",{"time", "velX", "velY", "velZ", "wRoll", "wPitch", "wYaw", "wr1", "wr2", "wr3", "wr4", "qwbi", "qwbj", "qwbk", "qwb0", "X", "Y", "Z", "windX", "windY", "windZ", "driftRoll", "driftPitch", "driftYaw","camScale","abiasX","abiasY","abiasZ"});
                log::filelogger reality("reality.log",{"time", "velX", "velY", "velZ", "wRoll", "wPitch", "wYaw", "wr1", "wr2", "wr3", "wr4", "qwbi", "qwbj", "qwbk", "qwb0", "X", "Y", "Z", "driftRoll", "driftPitch", "driftYaw", "windX", "windY", "windZ"});
            }
        #endif

        boost::mt19937 rng(std::time(0));
        boost::normal_distribution<> nd(0.0,1.0);
        boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor(rng, nd);

        YAML::Node config;

        #ifdef CRAP_PLOT
            using namespace cpplot;

            template<int rows, int cols>
            int index(int row, int col) { return row + (col-1)*rows; }

            static const int count = 30*10;
            #define rows 5
            #define cols 4
            auto xplot = figure("Reality")->subplot(rows,cols,index<rows,cols>(1,1))->title("Position X")->add<Line>()->set_capacity(count)->set("r");
            auto oxplot = figure("Reality")->subplot(rows,cols,index<rows,cols>(1,1))->add<Line>()->set_capacity(count)->set("b");
            auto yplot = figure("Reality")->subplot(rows,cols,index<rows,cols>(2,1))->title("Position Y")->add<Line>()->set_capacity(count)->set("r");
            auto oyplot = figure("Reality")->subplot(rows,cols,index<rows,cols>(2,1))->add<Line>()->set_capacity(count)->set("b");
            auto zplot = figure("Reality")->subplot(rows,cols,index<rows,cols>(3,1))->title("Position Z")->add<Line>()->set_capacity(count)->set("r");
            auto ozplot = figure("Reality")->subplot(rows,cols,index<rows,cols>(3,1))->add<Line>()->set_capacity(count)->set("b");

            auto vxplot = figure("Reality")->subplot(rows,cols,index<rows,cols>(1,2))->title("Velocity X")->add<Line>()->set_capacity(count)->set("r");
            auto ovxplot = figure("Reality")->subplot(rows,cols,index<rows,cols>(1,2))->add<Line>()->set_capacity(count)->set("b");
            auto vyplot = figure("Reality")->subplot(rows,cols,index<rows,cols>(2,2))->title("Velocity Y")->add<Line>()->set_capacity(count)->set("r");
            auto ovyplot = figure("Reality")->subplot(rows,cols,index<rows,cols>(2,2))->add<Line>()->set_capacity(count)->set("b");
            auto vzplot = figure("Reality")->subplot(rows,cols,index<rows,cols>(3,2))->title("Velocity Z")->add<Line>()->set_capacity(count)->set("r");
            auto ovzplot = figure("Reality")->subplot(rows,cols,index<rows,cols>(3,2))->add<Line>()->set_capacity(count)->set("b");

            auto rollplot = figure("Reality")->subplot(rows,cols,index<rows,cols>(1,3))->title("Roll")->add<Line>()->set_capacity(count)->set("r");
            auto pitchplot = figure("Reality")->subplot(rows,cols,index<rows,cols>(2,3))->title("Pitch")->add<Line>()->set_capacity(count)->set("r");
            auto yawplot = figure("Reality")->subplot(rows,cols,index<rows,cols>(3,3))->title("Yaw")->add<Line>()->set_capacity(count)->set("r");
            auto orollplot = figure("Reality")->subplot(rows,cols,index<rows,cols>(1,3))->add<Line>()->set_capacity(count)->set("b");
            auto opitchplot = figure("Reality")->subplot(rows,cols,index<rows,cols>(2,3))->add<Line>()->set_capacity(count)->set("b");
            auto oyawplot = figure("Reality")->subplot(rows,cols,index<rows,cols>(3,3))->add<Line>()->set_capacity(count)->set("b");

            auto wrollplot = figure("Reality")->subplot(rows,cols,index<rows,cols>(1,4))->title("Roll rate")->add<Line>()->set_capacity(count)->set("r");
            auto wpitchplot = figure("Reality")->subplot(rows,cols,index<rows,cols>(2,4))->title("Pitch rate")->add<Line>()->set_capacity(count)->set("r");
            auto wyawplot = figure("Reality")->subplot(rows,cols,index<rows,cols>(3,4))->title("Yaw rate")->add<Line>()->set_capacity(count)->set("r");
            auto owrollplot = figure("Reality")->subplot(rows,cols,index<rows,cols>(1,4))->add<Line>()->set_capacity(count)->set("b");
            auto owpitchplot = figure("Reality")->subplot(rows,cols,index<rows,cols>(2,4))->add<Line>()->set_capacity(count)->set("b");
            auto owyawplot = figure("Reality")->subplot(rows,cols,index<rows,cols>(3,4))->add<Line>()->set_capacity(count)->set("b");

            #undef rows
            #undef cols

            #define rows 5
            #define cols 2

            auto wplot1 = figure("Reality")->subplot(rows,cols,index<rows,cols>(4,1))->title("Wind")->add<Line>()->set("r")->set_capacity(count);
            auto wplot2 = figure("Reality")->subplot(rows,cols,index<rows,cols>(4,1))->add<Line>()->set("g")->set_capacity(count);
            auto wplot3 = figure("Reality")->subplot(rows,cols,index<rows,cols>(4,1))->add<Line>()->set("b")->set_capacity(count);
            auto owplot1 = figure("Reality")->subplot(rows,cols,index<rows,cols>(4,1))->add<Line>()->set("r")->set("- -")->set_capacity(count);
            auto owplot2 = figure("Reality")->subplot(rows,cols,index<rows,cols>(4,1))->add<Line>()->set("g")->set("- -")->set_capacity(count);
            auto owplot3 = figure("Reality")->subplot(rows,cols,index<rows,cols>(4,1))->add<Line>()->set("b")->set("- -")->set_capacity(count);

            auto orplot1 = figure("Reality")->subplot(rows,cols,index<rows,cols>(5,1))->title("omega_i")->add<Line>()->set("r")->set_capacity(count);
            auto orplot2 = figure("Reality")->subplot(rows,cols,index<rows,cols>(5,1))->add<Line>()->set("b")->set_capacity(count);
            auto orplot3 = figure("Reality")->subplot(rows,cols,index<rows,cols>(5,1))->add<Line>()->set("g")->set_capacity(count);
            auto orplot4 = figure("Reality")->subplot(rows,cols,index<rows,cols>(5,1))->add<Line>()->set("k")->set_capacity(count);

            auto qplot0 = figure("Reality")->subplot(rows,cols,index<rows,cols>(4,2))->title("q")->add<Line>()->set("k")->set_capacity(count);
            auto qploti = figure("Reality")->subplot(rows,cols,index<rows,cols>(4,2))->add<Line>()->set("r")->set_capacity(count);
            auto qplotj = figure("Reality")->subplot(rows,cols,index<rows,cols>(4,2))->add<Line>()->set("g")->set_capacity(count);
            auto qplotk = figure("Reality")->subplot(rows,cols,index<rows,cols>(4,2))->add<Line>()->set("b")->set_capacity(count);
            auto oqplot0 = figure("Reality")->subplot(rows,cols,index<rows,cols>(4,2))->add<Line>()->set("k")->set("- -")->set_capacity(count);
            auto oqploti = figure("Reality")->subplot(rows,cols,index<rows,cols>(4,2))->add<Line>()->set("r")->set("- -")->set_capacity(count);
            auto oqplotj = figure("Reality")->subplot(rows,cols,index<rows,cols>(4,2))->add<Line>()->set("g")->set("- -")->set_capacity(count);
            auto oqplotk = figure("Reality")->subplot(rows,cols,index<rows,cols>(4,2))->add<Line>()->set("b")->set("- -")->set_capacity(count);

            auto obxplot = figure("Reality")->subplot(rows,cols,index<rows,cols>(5,2))->title("Gyro bias")->add<Line>()->set_capacity(count)->set("r");
            auto obyplot = figure("Reality")->subplot(rows,cols,index<rows,cols>(5,2))->add<Line>()->set_capacity(count)->set("g");
            auto obzplot = figure("Reality")->subplot(rows,cols,index<rows,cols>(5,2))->add<Line>()->set_capacity(count)->set("b");


            #undef rows
            #undef cols

            #define rows 3
            #define cols 2

            auto Pxplot = figure("PTAM")->subplot(rows,cols,index<rows,cols>(1,1))->title("Position X")->add<Line>()->set("r")->set_capacity(count);
            auto Pyplot = figure("PTAM")->subplot(rows,cols,index<rows,cols>(2,1))->title("Position Y")->add<Line>()->set("r")->set_capacity(count);
            auto Pzplot = figure("PTAM")->subplot(rows,cols,index<rows,cols>(3,1))->title("Position Z")->add<Line>()->set("r")->set_capacity(count);
            auto PRplot = figure("PTAM")->subplot(rows,cols,index<rows,cols>(1,2))->title("Roll")->add<Line>()->set("r")->set_capacity(count);
            auto PPplot = figure("PTAM")->subplot(rows,cols,index<rows,cols>(2,2))->title("Pitch")->add<Line>()->set("r")->set_capacity(count);
            auto PYplot = figure("PTAM")->subplot(rows,cols,index<rows,cols>(3,2))->title("Yaw")->add<Line>()->set("r")->set_capacity(count);
            auto oPxplot = figure("PTAM")->subplot(rows,cols,index<rows,cols>(1,1))->add<Line>()->set("b")->set_capacity(count);
            auto oPyplot = figure("PTAM")->subplot(rows,cols,index<rows,cols>(2,1))->add<Line>()->set("b")->set_capacity(count);
            auto oPzplot = figure("PTAM")->subplot(rows,cols,index<rows,cols>(3,1))->add<Line>()->set("b")->set_capacity(count);
            auto oPRplot = figure("PTAM")->subplot(rows,cols,index<rows,cols>(1,2))->add<Line>()->set("b")->set_capacity(count);
            auto oPPplot = figure("PTAM")->subplot(rows,cols,index<rows,cols>(2,2))->add<Line>()->set("b")->set_capacity(count);
            auto oPYplot = figure("PTAM")->subplot(rows,cols,index<rows,cols>(3,2))->add<Line>()->set("b")->set_capacity(count);

            #undef rows
            #undef cols

            //~ #define rows 6
            //~ #define cols 1
//~
            //~ auto oqPw0plot = figure("PTAM")->subplot(rows,cols,index<rows,cols>(4,1))->title("qPw")->add<Line>()->set("k")->set("- -")->set_capacity(count);
            //~ auto oqPwiplot = figure("PTAM")->subplot(rows,cols,index<rows,cols>(4,1))->add<Line>()->set("r")->set("- -")->set_capacity(count);
            //~ auto oqPwjplot = figure("PTAM")->subplot(rows,cols,index<rows,cols>(4,1))->add<Line>()->set("g")->set("- -")->set_capacity(count);
            //~ auto oqPwkplot = figure("PTAM")->subplot(rows,cols,index<rows,cols>(4,1))->add<Line>()->set("b")->set("- -")->set_capacity(count);
//~
            //~ auto ocamXplot = figure("PTAM")->subplot(rows,cols,index<rows,cols>(5,1))->title("Origin")->add<Line>()->set("r")->set("- -")->set_capacity(count);
            //~ auto ocamYplot = figure("PTAM")->subplot(rows,cols,index<rows,cols>(5,1))->add<Line>()->set("g")->set("- -")->set_capacity(count);
            //~ auto ocamZplot = figure("PTAM")->subplot(rows,cols,index<rows,cols>(5,1))->add<Line>()->set("b")->set("- -")->set_capacity(count);
//~
            //~ auto ocamsplot = figure("PTAM")->subplot(rows,cols,index<rows,cols>(6,1))->title("Scale")->add<Line>()->set("b")->set("- -")->set_capacity(count);
//~
            //~ #undef rows
            //~ #undef cols
        #endif


        namespace imu {
            using namespace LinkQuad::comm::serial::data;
            using namespace LinkQuad::comm::serial::data::SSMCU;
            CRAP::sensors::imu::serial_data measurement;

            float scales[4];

            void add_some_imu_measurement_noise(Matrix<float, model::imu::data_size, 1>& imu_data) {
                imu_data(model::imu::accelerometers[X]) += config["imu_noise"]["ax"].as<base_float_t>(0.01) * var_nor();
                imu_data(model::imu::accelerometers[Y]) += config["imu_noise"]["ay"].as<base_float_t>(0.01) * var_nor();
                imu_data(model::imu::accelerometers[Z]) += config["imu_noise"]["az"].as<base_float_t>(0.01) * var_nor();

                imu_data(model::imu::gyroscopes[X]) += config["imu_noise"]["wx"].as<base_float_t>(0.01) * var_nor();
                imu_data(model::imu::gyroscopes[Y]) += config["imu_noise"]["wy"].as<base_float_t>(0.01) * var_nor();
                imu_data(model::imu::gyroscopes[Z]) += config["imu_noise"]["wz"].as<base_float_t>(0.01) * var_nor();

                //~ imu_data(model::imu::magnetometers[X]) += config["imu_noise"]["wx"].as<base_float_t>(0.01) * var_nor();
                //~ imu_data(model::imu::magnetometers[Y]) += config["imu_noise"]["wy"].as<base_float_t>(0.01) * var_nor();
                //~ imu_data(model::imu::magnetometers[Z]) += config["imu_noise"]["wz"].as<base_float_t>(0.01) * var_nor();

                imu_data(model::imu::pressure) += config["imu_noise"]["alt"].as<base_float_t>(0.0042) * var_nor();
            }
            void simulate() {
                Matrix<float, model::imu::data_size, 1> imu_data(model::imu::measurement(x).cast<float>());
                //~ add_some_imu_measurement_noise(imu_data);

                measurement.accel_raw_0 = imu_data(model::imu::accelerometers[X]) / scales[0];
                measurement.accel_raw_1 = imu_data(model::imu::accelerometers[Y]) / scales[0];
                measurement.accel_raw_2 = imu_data(model::imu::accelerometers[Z]) / scales[0];

                measurement.gyro_data_0 = imu_data(model::imu::gyroscopes[X]) / scales[1];
                measurement.gyro_data_1 = imu_data(model::imu::gyroscopes[Y]) / scales[1];
                measurement.gyro_data_2 = imu_data(model::imu::gyroscopes[Z]) / scales[1];

                //~ measurement.micromag_0 = imu_data(model::imu::magnetometers[X]) / scales[2];
                //~ measurement.micromag_1 = imu_data(model::imu::magnetometers[Y]) / scales[2];
                //~ measurement.micromag_2 = imu_data(model::imu::magnetometers[Z]) / scales[2];

                measurement.fAlt = imu_data(model::imu::pressure) / scales[3];

                measurement.system_status = 1;
                measurement.battery_voltage = 10;

                CRAP::comm::send(config["channels"]["imu"].as<std::string>("/imu_serial"), measurement);
            }
        }

        namespace camera {
            //~ void plot_world(
                //~ const Eigen::Matrix<double, 3, 1>& XPTAM,
                //~ const Eigen::Quaternion<double>& qPTAM,
                //~ const Eigen::Quaternion<double>& qbc,
                //~ const Eigen::Quaternion<double>& qwb,
                //~ const Eigen::Matrix<double, 3, 1>& origin,
                //~ const Eigen::Quaternion<double>& qPw,
                //~ const double scale)
            //~ {
                //~ using namespace cpplot; double t = starting_time.elapsed();
                //~ Eigen::Matrix<double, 3, 1> X = origin + qPw.toRotationMatrix().transpose()*XPTAM/scale - qwb.toRotationMatrix() * model::camera::parameters::camera_position;
                //~ Eigen::Quaternion<double> q = qPw.inverse() * qPTAM * qbc.inverse();
//~
                //~ Eigen::Vector3d eul; eul << quat2eul(q.w(),q.x(),q.y(),q.z());
//~
                //~ oPxplot << std::make_pair(t, X.x());
                //~ oPyplot << std::make_pair(t, X.y());
                //~ oPzplot << std::make_pair(t, X.z());
                //~ oPRplot << std::make_pair(t, (double)eul[0]);
                //~ oPPplot << std::make_pair(t, (double)eul[1]);
                //~ oPYplot << std::make_pair(t, (double)eul[2]);
            //~ }
            sensors::camera::serial_data serial_camera_data;
            void add_some_camera_measurement_noise(Matrix<base_float_t, model::camera::data_size, 1>& meas) {
                double s = config["camera_noise"]["q"].as<base_float_t>(0.01);
                double p = config["camera_noise"]["position"].as<base_float_t>(0.05);
                meas(model::camera::quaternion_part_vector[0]) += s * var_nor();
                meas(model::camera::quaternion_part_vector[1]) += s * var_nor();
                meas(model::camera::quaternion_part_vector[2]) += s * var_nor();
                meas(model::camera::quaternion_part_real)      += s * var_nor();
                meas(model::camera::positions[X])      += p * var_nor();
                meas(model::camera::positions[Y])      += p * var_nor();
                meas(model::camera::positions[Z])      += p * var_nor();
            }
            void simulate() {
                static bool first = true;

                Eigen::Quaternion<double> qbc;
                    qbc =
                        Eigen::AngleAxis<double>(-30*M_PI/180, Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxis<double>(M_PI_2, Eigen::Vector3d::UnitZ())
                        * Eigen::AngleAxis<double>(M_PI_2, Eigen::Vector3d::UnitX());

                if(first) {
                    Eigen::Matrix<double, 3, 1> XPTAM; XPTAM << -0.0454242, -1.29609, 0.761661;
                    Eigen::Quaternion<double> qPTAM;
                    qPTAM.x() =  0.868049;
                    qPTAM.y() =  0.00592789;
                    qPTAM.z() =  -0.0150805;
                    qPTAM.w() =  -0.496213;
                    //~ model::camera::initialize(x,XPTAM,qPTAM,qbc,true);
                    Map<Quaternion<scalar> > qwb(&x.data()[state::quaternion]);
                    CRAP::model::camera::initialize(x, XPTAM, qPTAM, qbc);
                    //~ CRAP::model::camera::initialize(x, XPTAM, qPTAM, qbc,
                        //~ (Vector3d() << config["camera"]["init_pos"][0].as<scalar>(),config["camera"]["init_pos"][1].as<scalar>(),config["camera"]["init_pos"][2].as<scalar>()).finished()
                         //~ + qwb.toRotationMatrix()*model::camera::parameters::camera_position,
                        //~ true);
                    first = false;
                }
                Matrix<base_float_t, model::camera::data_size, 1> meas(model::camera::measurement(x));

                //~ {
                    //~ Vector3d XPTAM = meas.segment<3>(model::camera::position);
                    //~ Quaterniond qPTAM(
                        //~ meas(model::camera::quaternion_part_real),
                        //~ meas(model::camera::quaternion_part_vector[0]),
                        //~ meas(model::camera::quaternion_part_vector[1]),
                        //~ meas(model::camera::quaternion_part_vector[2])
                    //~ );
                    //~ Quaterniond qPw(
                        //~ x(state::qPw_real),
                        //~ x(state::qPw_vector[0]),
                        //~ x(state::qPw_vector[1]),
                        //~ x(state::qPw_vector[2])
                    //~ );
                    //~ Vector3d origin = x.segment<3>(state::camera_origin);
//~
                    //~ #ifdef CRAP_PLOT
                        //~ const Quaternion<scalar> qwb(
                            //~ x(state::quaternion_part_real),
                            //~ x(state::quaternion_part_vector[0]),
                            //~ x(state::quaternion_part_vector[1]),
                            //~ x(state::quaternion_part_vector[2])
                        //~ );
                        //~ using namespace cpplot; double t = starting_time.elapsed();
                        //~ plot_world(
                            //~ XPTAM,
                            //~ qPTAM,
                            //~ qbc,
                            //~ qwb,
                            //~ origin,
                            //~ qPw,
                            //~ x(state::camera_scale)
                        //~ );
                        //~ Eigen::Vector3d eul; eul << quat2eul(x(state::quaternion_part_real),x(state::quaternion_part_vector[X]),x(state::quaternion_part_vector[Y]),x(state::quaternion_part_vector[Z]));
//~
                        //~ Pxplot << std::make_pair(t, x(state::positions[X]));
                        //~ Pyplot << std::make_pair(t, x(state::positions[Y]));
                        //~ Pzplot << std::make_pair(t, x(state::positions[Z]));
                        //~ PRplot << std::make_pair(t, (double)eul[0]);
                        //~ PPplot << std::make_pair(t, (double)eul[1]);
                        //~ PYplot << std::make_pair(t, (double)eul[2]);
                    //~ #endif
                //~ }

                //~ add_some_camera_measurement_noise(meas);
                serial_camera_data.params_32f_0 = meas(model::camera::quaternion_part_vector[0]);
                serial_camera_data.params_32f_1 = meas(model::camera::quaternion_part_vector[1]);
                serial_camera_data.params_32f_2 = meas(model::camera::quaternion_part_vector[2]);
                serial_camera_data.params_32f_3 = meas(model::camera::quaternion_part_real);
                serial_camera_data.params_32f_4 = meas(model::camera::positions[X]);
                serial_camera_data.params_32f_5 = meas(model::camera::positions[Y]);
                serial_camera_data.params_32f_6 = meas(model::camera::positions[Z]);
                serial_camera_data.params_8i_0 = sensors::camera::quality::GOOD;
                CRAP::comm::send(config["channels"]["camera"].as<std::string>("/camera_serial"), serial_camera_data);
            }
        }

        namespace simulation {
            void simulate_wind(state_vector& xdot) {
                using std::sin;
                double t = CRAP::starting_time.elapsed();
                xdot(state::wind_velocities[X]) = config["wind"]["strength"]["x"].as<double>(0.58) * sin(2 * (M_PI / config["wind"]["period"]["x"].as<double>(0.1)) * t + config["wind"]["phase"]["x"].as<double>(0.0));
                xdot(state::wind_velocities[Y]) = config["wind"]["strength"]["y"].as<double>(0.58) * sin(2 * (M_PI / config["wind"]["period"]["y"].as<double>(0.1)) * t + config["wind"]["phase"]["y"].as<double>(0.0));
                xdot(state::wind_velocities[Z]) = config["wind"]["strength"]["z"].as<double>(0.58) * sin(2 * (M_PI / config["wind"]["period"]["z"].as<double>(0.1)) * t + config["wind"]["phase"]["z"].as<double>(0.0));
            }
            void randomize(state_vector& xdot) {
                xdot(state::velocities[X]) += config["randomness"]["vx"].as<base_float_t>(0.01) * var_nor();
                xdot(state::velocities[Y]) += config["randomness"]["vy"].as<base_float_t>(0.01) * var_nor();
                xdot(state::velocities[Z]) += config["randomness"]["vz"].as<base_float_t>(0.01) * var_nor();
                xdot(state::omega[X]) += config["randomness"]["wx"].as<base_float_t>(0.01) * var_nor();
                xdot(state::omega[Y]) += config["randomness"]["wy"].as<base_float_t>(0.01) * var_nor();
                xdot(state::omega[Z]) += config["randomness"]["wz"].as<base_float_t>(0.01) * var_nor();
                xdot(state::omega_r[0]) += config["randomness"]["wi"].as<base_float_t>(0.01) * var_nor();
                xdot(state::omega_r[1]) += config["randomness"]["wi"].as<base_float_t>(0.01) * var_nor();
                xdot(state::omega_r[2]) += config["randomness"]["wi"].as<base_float_t>(0.01) * var_nor();
                xdot(state::omega_r[3]) += config["randomness"]["wi"].as<base_float_t>(0.01) * var_nor();

                xdot(state::quaternion_part_real) += config["randomness"]["q0"].as<base_float_t>(0.001) * var_nor();
                xdot(state::quaternion_part_vector[X]) += config["randomness"]["qi"].as<base_float_t>(0.001) * var_nor();
                xdot(state::quaternion_part_vector[Y]) += config["randomness"]["qj"].as<base_float_t>(0.001) * var_nor();
                xdot(state::quaternion_part_vector[Z]) += config["randomness"]["qk"].as<base_float_t>(0.001) * var_nor();

                xdot(state::positions[X]) += config["randomness"]["x"].as<base_float_t>(0.01) * var_nor();
                xdot(state::positions[Y]) += config["randomness"]["y"].as<base_float_t>(0.01) * var_nor();
                xdot(state::positions[Z]) += config["randomness"]["z"].as<base_float_t>(0.01) * var_nor();

                xdot(state::wind_velocities[X]) += config["randomness"]["windx"].as<base_float_t>(0.1) * var_nor();
                xdot(state::wind_velocities[Y]) += config["randomness"]["windy"].as<base_float_t>(0.1) * var_nor();
                xdot(state::wind_velocities[Z]) += config["randomness"]["windz"].as<base_float_t>(0.1) * var_nor();

                xdot(state::gyro_offsets[X]) += config["randomness"]["gyrox"].as<base_float_t>(0.01) * var_nor();
                xdot(state::gyro_offsets[Y]) += config["randomness"]["gyroy"].as<base_float_t>(0.01) * var_nor();
                xdot(state::gyro_offsets[Z]) += config["randomness"]["gyroz"].as<base_float_t>(0.01) * var_nor();
//~
                //~ xdot(state::qPw_real) += config["randomness"]["qPw0"].as<base_float_t>(0.001) * var_nor();
                //~ xdot(state::qPw_vector[X]) += config["randomness"]["qPwi"].as<base_float_t>(0.001) * var_nor();
                //~ xdot(state::qPw_vector[Y]) += config["randomness"]["qPwj"].as<base_float_t>(0.001) * var_nor();
                //~ xdot(state::qPw_vector[Z]) += config["randomness"]["qPwk"].as<base_float_t>(0.001) * var_nor();
//~
                //~ xdot(state::camera_origins[X]) += config["randomness"]["camera_origin_x"].as<base_float_t>(0.01) * var_nor();
                //~ xdot(state::camera_origins[Y]) += config["randomness"]["camera_origin_y"].as<base_float_t>(0.01) * var_nor();
                //~ xdot(state::camera_origins[Z]) += config["randomness"]["camera_origin_z"].as<base_float_t>(0.01) * var_nor();
                //~
                //~ xdot(state::camera_origins[Z]) += config["randomness"]["camera_scale"].as<base_float_t>(0.01) * var_nor();
            }
            void clamp(state_vector& xdot) {
                using std::min;
                if(x(state::positions[Z]) > -0.4) {
                    xdot(state::positions[Z]) = min(0.0, xdot(state::positions[Z]));
                    xdot(state::velocities[Z]) = min(0.0, xdot(state::velocities[Z]));
                    std::cout << "Landed (clamped)" << std::endl;
                }
                for(int i = 0; i < 4; ++i) {
                    if(x(state::omega_r[i]) < 0) x(state::omega_r[i]) = 0;
                    else if(x(state::omega_r[i]) > 1500) x(state::omega_r[i]) = 1500;
                }
            }

            state_vector& simulate(state_vector& xdot) {
                //~ simulate_wind(xdot);
                //~ randomize(xdot);
                //~ clamp(xdot);
                return xdot;
            }

            state_vector xdot;
            const state_vector& f_continuous(const state_vector& x, const control_vector& u) {
                xdot = model::fc(x,u);
                return simulate(xdot);
            }
        }

        void measure() {
            static int i = 0;
            if(config["simulate_imu"].as<bool>(true)) imu::measure();

            if(config["simulate_camera"].as<bool>(true)) {
                i = (i+1) % 30;
                if(i == 0) camera::measure();
            }

            CRAP::comm::send(config["channels"]["reality"].as<std::string>("/reality"), x);
        }

        void time_update() {
            u = ctrl_signal();
            //~ x = math::rk4(simulation::f_continuous, x, u, model::T);
            x += model::T * simulation::f_continuous(x,u);
            x.segment<4>(state::quaternion).normalize();
            measure();


            #if defined(CRAP_PLOT) || defined(CRAP_OUTPUT_DATA)
                double t = CRAP::starting_time.elapsed();
            #endif
            #ifdef CRAP_PLOT
                using namespace cpplot;
                xplot << std::make_pair(t, x(state::positions[X]));
                yplot << std::make_pair(t, x(state::positions[Y]));
                zplot << std::make_pair(t, x(state::positions[Z]));

                vxplot << std::make_pair(t, x(state::velocities[X]));
                vyplot << std::make_pair(t, x(state::velocities[Y]));
                vzplot << std::make_pair(t, x(state::velocities[Z]));

                //~ wplot1 << std::make_pair(t, x(state::omega_r[0]));
                //~ wplot2 << std::make_pair(t, x(state::omega_r[1]));
                //~ wplot3 << std::make_pair(t, x(state::omega_r[2]));
                //~ wplot4 << std::make_pair(t, x(state::omega_r[3]));
                Vector3d eul; eul << quat2eul(
                    x(state::quaternion_part_real),
                    x(state::quaternion_part_vector[X]),
                    x(state::quaternion_part_vector[Y]),
                    x(state::quaternion_part_vector[Z])
                );

                rollplot << std::make_pair(t, eul[X]);
                pitchplot << std::make_pair(t, eul[Y]);
                yawplot << std::make_pair(t, eul[Z]);

                wrollplot << std::make_pair(t, x(state::omega[X]));
                wpitchplot << std::make_pair(t, x(state::omega[Y]));
                wyawplot << std::make_pair(t, x(state::omega[Z]));

                wplot1 << std::make_pair(t, x(state::wind_velocities[0]));
                wplot2 << std::make_pair(t, x(state::wind_velocities[1]));
                wplot3 << std::make_pair(t, x(state::wind_velocities[2]));

                qplot0 << std::make_pair(t, x(state::quaternion_part_real));
                qploti << std::make_pair(t, x(state::quaternion_part_vector[0]));
                qplotj << std::make_pair(t, x(state::quaternion_part_vector[1]));
                qplotk << std::make_pair(t, x(state::quaternion_part_vector[2]));
            #endif

            #ifdef CRAP_OUTPUT_DATA
                output::reality(t, x);
            #endif
        }

        #ifdef CRAP_PLOT
        void plot_observer(const state_vector& x) {
            using namespace cpplot; double t = CRAP::starting_time.elapsed();
            oxplot << std::make_pair(t, x(state::positions[X]));
            oyplot << std::make_pair(t, x(state::positions[Y]));
            ozplot << std::make_pair(t, x(state::positions[Z]));

            ovxplot << std::make_pair(t, x(state::velocities[X]));
            ovyplot << std::make_pair(t, x(state::velocities[Y]));
            ovzplot << std::make_pair(t, x(state::velocities[Z]));

            Vector3d eul; eul << quat2eul(
                x(state::quaternion_part_real),
                x(state::quaternion_part_vector[X]),
                x(state::quaternion_part_vector[Y]),
                x(state::quaternion_part_vector[Z])
            );

            orollplot << std::make_pair(t, eul[X]);
            opitchplot << std::make_pair(t, eul[Y]);
            oyawplot << std::make_pair(t, eul[Z]);

            owrollplot << std::make_pair(t, x(state::omega[X]));
            owpitchplot << std::make_pair(t, x(state::omega[Y]));
            owyawplot << std::make_pair(t, x(state::omega[Z]));

            owplot1 << std::make_pair(t, x(state::wind_velocities[0]));
            owplot2 << std::make_pair(t, x(state::wind_velocities[1]));
            owplot3 << std::make_pair(t, x(state::wind_velocities[2]));

            orplot1 << std::make_pair(t, -x(state::omega_r[0]));
            orplot2 << std::make_pair(t, x(state::omega_r[1]));
            orplot3 << std::make_pair(t, -x(state::omega_r[2]));
            orplot4 << std::make_pair(t, x(state::omega_r[3]));

            oqplot0 << std::make_pair(t, x(state::quaternion_part_real));
            oqploti << std::make_pair(t, x(state::quaternion_part_vector[0]));
            oqplotj << std::make_pair(t, x(state::quaternion_part_vector[1]));
            oqplotk << std::make_pair(t, x(state::quaternion_part_vector[2]));

            obxplot << std::make_pair(t, x(state::gyro_offsets[X]));
            obyplot << std::make_pair(t, x(state::gyro_offsets[Y]));
            obzplot << std::make_pair(t, x(state::gyro_offsets[Z]));

            //~ oqPw0plot << std::make_pair(t, x(state::qPw_real));
            //~ oqPwiplot << std::make_pair(t, x(state::qPw_vector[0]));
            //~ oqPwjplot << std::make_pair(t, x(state::qPw_vector[1]));
            //~ oqPwkplot << std::make_pair(t, x(state::qPw_vector[2]));

            //~ ocamXplot <<  std::make_pair(t, x(state::camera_origins[0]));
            //~ ocamYplot <<  std::make_pair(t, x(state::camera_origins[1]));
            //~ ocamZplot <<  std::make_pair(t, x(state::camera_origins[2]));

            //~ ocamsplot <<  std::make_pair(t, x(state::camera_scale));
        }
        #endif

        void init() {
            x.setZero();
            x.segment<3>(state::position) <<
                config["initial_state"]["position"][0].as<state::scalar>(0.0),
                config["initial_state"]["position"][1].as<state::scalar>(0.0),
                config["initial_state"]["position"][2].as<state::scalar>(0.0);

            x.segment<4>(state::quaternion) << eul2quat(config["initial_state"]["orientation"][0].as<state::scalar>(0.0), config["initial_state"]["orientation"][1].as<state::scalar>(0.0), config["initial_state"]["orientation"][2].as<state::scalar>(0.0));

            x.segment<4>(state::rotor_velocities) <<
                -config["initial_state"]["rotor_velocities"].as<state::scalar>(0.0), // Forward
                config["initial_state"]["rotor_velocities"].as<state::scalar>(0.0), // Left
                -config["initial_state"]["rotor_velocities"].as<state::scalar>(0.0), // Back
                config["initial_state"]["rotor_velocities"].as<state::scalar>(0.0); // Right

            std::cout << "Real starting state: " << x.transpose() << std::endl;


            ctrl_signal = comm::bind<controller::control_signal_fn>("controller", "control_signal");
        }
    }
}

extern "C" {
    using namespace CRAP::Reality;
    state_vector get_state() {
        return x;
    }
}

extern "C" {
    using namespace CRAP;
    using namespace CRAP::Reality;
    void configure(YAML::Node& c) {
        std::cout << "Reconfiguring Reality: " << std::endl << c << std::endl;
        config = c;
        Reality::imu::scales[0] = c["scales"]["accelerometer"].as<float>(1.0);
        Reality::imu::scales[1] = c["scales"]["gyro"].as<float>(1.0);
        Reality::imu::scales[2] = c["scales"]["magnetometer"].as<float>(1.0);
        Reality::imu::scales[3] = c["scales"]["altitude"].as<float>(1.0);
    }

    bool running = true;
    time::frequency_t frequency(model::frequency);

    void run() {
        Reality::init();

        #ifdef CRAP_PLOT
            CRAP::comm::listen("/state_estimate", plot_observer);
        #endif

        while(running && time::ticktock(::frequency)) {
            Reality::time_update();
        }
    }

    void stop() {
        running = false;
    }
}
