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
#include "sensor_data.hpp"
#include "modules/model/model.hpp"
#include "linkquad/serial_communication.hpp"

#include <iostream>
#include <iomanip>
#include "math/filtering/filtering.hpp"

namespace CRAP {
    namespace sensor_reader {
        using namespace CRAP::sensors;
        using model::imu::scalar;
        using namespace Eigen;
        YAML::Node config;
        filtering::observation<model::imu::data_size> measurement;
        Matrix<scalar, model::imu::data_size, 1> scales;

        using namespace LinkQuad::comm::serial::data;
        using namespace LinkQuad::comm::serial::data::SSMCU;
        void smcu(const imu::serial_data& d) {

            measurement.z <<
                (scalar)d.accel_raw_0,
                (scalar)d.accel_raw_1,
                (scalar)d.accel_raw_2,

                (scalar)d.gyro_data_0,
                (scalar)d.gyro_data_1,
                (scalar)d.gyro_data_2

                //~ (scalar)d.fAlt

                //~ (scalar)d.micromag_0,
                //~ (scalar)d.micromag_1,
                //~ (scalar)d.micromag_2
            ;

            //~ std::cout << "Received: " << measurement.z.transpose() << std::endl;

            measurement.z.array() *= scales.array();
            //~ measurement.z.segment<3>(model::imu::magnetometer).normalize();
            //~ std::cout << "Scaled: " << measurement.z.transpose() << std::endl;

            CRAP::comm::send(config["channels"]["imu"].as<std::string>("/imu"), measurement);
        }
    }
}


extern "C" {
    using namespace CRAP;
    using namespace CRAP::sensor_reader;
    using namespace LinkQuad::comm::serial::data;
    void configure(YAML::Node& c) {
        using model::X;
        using model::Y;
        using model::Z;
        config = c;
        scales.segment<3>(model::imu::accelerometer).setConstant(c["scales"]["accelerometer"].as<base_float_t>(1.0));
        scales.segment<3>(model::imu::gyroscope).setConstant(c["scales"]["gyro"].as<base_float_t>(1.0));
        //~ scales(model::imu::pressure) = c["scales"]["pressure"].as<base_float_t>(1.0);
        //~ scales.segment<3>(model::imu::magnetometer).setConstant(c["scales"]["magnetometer"].as<base_float_t>(1.0));
        //~ scales(imu::battery) = c["scales"]["battery"].as<base_float_t>(1.0);

        measurement.R.setZero();
        measurement.R.diagonal()(model::imu::accelerometers[X]) = config["R"]["accelerometer"].as<base_float_t>(3e-1);
        measurement.R.diagonal()(model::imu::accelerometers[Y]) = config["R"]["accelerometer"].as<base_float_t>(3e-1);
        measurement.R.diagonal()(model::imu::accelerometers[Z]) = config["R"]["accelerometer"].as<base_float_t>(3e-1);

        measurement.R.diagonal()(model::imu::gyroscopes[X]) = config["R"]["gyroscopes"].as<base_float_t>(3e-1);
        measurement.R.diagonal()(model::imu::gyroscopes[Y]) = config["R"]["gyroscopes"].as<base_float_t>(3e-1);
        measurement.R.diagonal()(model::imu::gyroscopes[Z]) = config["R"]["gyroscopes"].as<base_float_t>(3e-1);

        //~ measurement.R.diagonal()(model::imu::pressure) = config["R"]["pressure"].as<base_float_t>(4e-3);

        //~ measurement.R.diagonal()(model::imu::magnetometers[X]) = config["R"]["magnetometers"][0].as<base_float_t>(3e-1);
        //~ measurement.R.diagonal()(model::imu::magnetometers[Y]) = config["R"]["magnetometers"][1].as<base_float_t>(3e-1);
        //~ measurement.R.diagonal()(model::imu::magnetometers[Z]) = config["R"]["magnetometers"][2].as<base_float_t>(3e-1);
    }

    void run() {
        if(config["use_smcu"].as<bool>(false)) {
            LinkQuad::comm::serial::listen<SSMCU::Part>(config["smcu_port"].as<std::string>(), smcu);
        } else {
            CRAP::comm::listen(config["smcu_topic"].as<std::string>("/imu_serial"), smcu);
        }
    }
}
