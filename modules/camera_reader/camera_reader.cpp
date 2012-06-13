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
#include "camera_data.hpp"
#include "modules/model/model.hpp"
#include "linkquad/serial_communication.hpp"
#include "math/math.hpp"

#include <iostream>
#include <iomanip>
#include "math/filtering/filtering.hpp"

namespace CRAP {
    namespace camera_reader {
        using namespace sensors::camera;
        using model::state_vector;
        using model::state_covariance_matrix;
        using namespace Eigen;
        typedef base_float_t scalar;
        typedef filtering::observation<model::camera::data_size> measurement_t;
        YAML::Node config;
        measurement_t measurement;
        measurement_t::covariance_matrix R[3];
        Matrix<scalar, model::camera::data_size, 1> scales;

        #ifdef CRAP_PLOT
            using namespace cpplot;
            static const int count = 30*10;
            auto xplot = figure("Camera")->subplot(2,3,1)->title("Position X")->add<Line>()->set_capacity(count);
            auto yplot = figure("Camera")->subplot(2,3,3)->title("Position Y")->add<Line>()->set_capacity(count);
            auto zplot = figure("Camera")->subplot(2,3,5)->title("Position Z")->add<Line>()->set_capacity(count);
            auto Rplot = figure("Camera")->subplot(2,3,2)->title("Roll")->add<Line>()->set_capacity(count);
            auto Pplot = figure("Camera")->subplot(2,3,4)->title("Pitch")->add<Line>()->set_capacity(count);
            auto Yplot = figure("Camera")->subplot(2,3,6)->title("Yaw")->add<Line>()->set_capacity(count);
        #endif

        using namespace LinkQuad::comm::serial::data;
        using namespace LinkQuad::comm::serial::data::SSMCU;
        void camera_receive(const sensors::camera::serial_data& d) {
            measurement.z <<
                (scalar) d.params_32f_0,
                (scalar) d.params_32f_1,
                (scalar) d.params_32f_2,
                (scalar) d.params_32f_3,
                (scalar) d.params_32f_4,
                (scalar) d.params_32f_5,
                (scalar) d.params_32f_6;

            int which;
            if( d.params_8i_0 == sensors::camera::quality::GOOD
             || d.params_8i_0 == sensors::camera::quality::DODGY
             || d.params_8i_0 == sensors::camera::quality::BAD)
            {
                which = d.params_8i_0;
            } else {
                which = sensors::camera::quality::BAD;
            }
            measurement.R = R[which];

            //~ std::cout << "Received: " << measurement.z.transpose() << std::endl;
            //~ std::cout << "Tracking quality: " << d.params_8i_0 << std::endl;

            #ifdef CRAP_PLOT
                using namespace cpplot; double t = CRAP::starting_time.elapsed();
                using model::X; using model::Y; using model::Z;
                Vector3d eul; eul << quat2eul(d.params_32f_3, d.params_32f_0, d.params_32f_1, d.params_32f_2);
//~ //~
                xplot << std::make_pair(t, measurement.z[model::camera::positions[X]]);
                yplot << std::make_pair(t, measurement.z[model::camera::positions[Y]]);
                zplot << std::make_pair(t, measurement.z[model::camera::positions[Z]]);
                Rplot << std::make_pair(t, eul[0]);
                Pplot << std::make_pair(t, eul[1]);
                Yplot << std::make_pair(t, eul[2]);
            #endif

            if(d.params_8i_0 == sensors::camera::quality::GOOD) {
                CRAP::comm::send(config["channels"]["camerapose"].as<std::string>("/pose"), measurement);
            }
        }
    }
}


extern "C" {
    using namespace CRAP;
    using namespace CRAP::camera_reader;
    using namespace LinkQuad::comm::serial::data;
    void configure(YAML::Node& c) {
        config = c;

        R[sensors::camera::quality::GOOD].setZero();
        R[sensors::camera::quality::GOOD].diagonal().segment<4>(model::camera::quaternion).setConstant(config["R"]["orientation"].as<base_float_t>(3e-3));
        R[sensors::camera::quality::GOOD].diagonal().segment<3>(model::camera::position).setConstant(config["R"]["position"].as<base_float_t>(3e-3));

        R[sensors::camera::quality::DODGY] = config["R_dodgy"].as<base_float_t>(2.0) * R[sensors::camera::quality::GOOD];
        R[sensors::camera::quality::DODGY] = config["R_bad"].as<base_float_t>(3.0) * R[sensors::camera::quality::GOOD];
    }

    void run() {
        if(config["go_serial"].as<bool>(false)) {
            LinkQuad::comm::serial::listen<SSMCU::Part>(config["serial_port"].as<std::string>(), camera_receive);
        } else {
            CRAP::comm::listen(config["camera_topic"].as<std::string>("/camera_serial"), camera_receive);
        }
    }
}
