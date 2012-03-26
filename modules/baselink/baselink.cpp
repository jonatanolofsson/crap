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
#include <iostream>
#include <string>
#include "linkquad/serial_communication.hpp"
#include "comm_messages.hpp"
#include "modules/controller/model.hpp"
#include "modules/model/model.hpp"



namespace CRAP {
    namespace baselink {
        using namespace Eigen;
        YAML::Node config;
        int reference_count = 0;
        boost::timer time_of_last_control;
        bool has_control = false;
        Matrix<float, 4, 1> joy;

        void receive_command(const ::CRAP::comm_messages::command_message d) {
            time_of_last_control.restart();
            if(BIT_TST(d.params_8i_0, ::CRAP::comm_messages::TRANSLATION_AND_ROTATION)) {
                joy <<
                    d.params_32f_0, d.params_32f_1, d.params_32f_2,
                    d.params_32f_3;
                ++reference_count;
                    //~ send_reference();
                //~ std::cout << "New reference: " << joy.transpose() << std::endl;
                has_control = true;
            }
            else if(BIT_TST(d.params_8i_0, ::CRAP::comm_messages::BUTTON_PRESS)) {
                std::cout << "Button press: " << (int)d.params_8i_1 << std::endl;
                CRAP::comm::send("/button_press", ::CRAP::comm_messages::button_event(true,d.params_8i_1));
            }
            else if(BIT_TST(d.params_8i_0, ::CRAP::comm_messages::BUTTON_RELEASE)) {
                std::cout << "Button release: " << (int)d.params_8i_1 << std::endl;
                CRAP::comm::send("/button_press", ::CRAP::comm_messages::button_event(false,d.params_8i_1));
            }
        }

        void button_action(const ::CRAP::comm_messages::button_event& e) {
            using CRAP::comm_messages::button_event;
            static bool press[2];
            press[e.button] = e.press;
            if(e.press) {
                if(e.button == button_event::BUTTONS::LEFT) {
                    CRAP::comm::send<std::string>("/logic/mode", "hover");
                }
                else if(e.button == button_event::BUTTONS::RIGHT) {
                    if(press[button_event::BUTTONS::LEFT]) {
                        CRAP::comm::send<std::string>("/logic/mode", "freeflight");
                    } else  {
                        CRAP::comm::send<std::string>("/logic/mode", "landing");
                    }
                }
            }
        }

        void check_connectivity() {
            //~ std::cout << "Has control: " << has_control << ". Elapsed time: " << time_of_last_control.elapsed() << std::endl;
            if(has_control && time_of_last_control.elapsed() > config["reference_timeout"].as<double>(1.0)) {
                has_control = false;
                //~ std::cout << "Reset control!" << std::endl;
                joy.setZero(); // FIXME: Better reference from config?
                time_of_last_control.restart();
                ++reference_count;
                //~ send_reference();
            }
        }

        comm_messages::state_message state_msg;
        void send_state(const observer::model::state_vector& x) {
            using namespace observer::model;
            state_msg.params_32f_0 = x(state::position[X]);
            state_msg.params_32f_1 = x(state::position[Y]);
            state_msg.params_32f_2 = x(state::position[Z]);

            state_msg.params_32f_3 = x(state::quaternion_part_real);
            state_msg.params_32f_4 = x(state::quaternion_part_vector[0]);
            state_msg.params_32f_5 = x(state::quaternion_part_vector[1]);
            state_msg.params_32f_6 = x(state::quaternion_part_vector[2]);
            LinkQuad::comm::serial::send(config["port"].as<std::string>(), state_msg);
        }
    }
}

extern "C" {
    bool has_new_reference(int& last) {
        return (last != CRAP::baselink::reference_count && (last = CRAP::baselink::reference_count));
    }
    Eigen::Matrix<float, 4, 1> get_reference() {
        return CRAP::baselink::joy;
    }
}

extern "C" {
    using namespace CRAP::baselink;
    using namespace CRAP::time;

    void configure(YAML::Node& c) {
        //~ std::cout << "Reconfiguring receiver" << std::endl
                //~ << c << std::endl;
        config = c;
    }
    void run() {
        if(config["port"]) {
            if(config["send_flight_data"].as<bool>(false)) {
                CRAP::comm::listen("/state_estimate", send_state);
            }
            if(config["receive_flight_commands"].as<bool>(false)) {
                std::cout << "Running receiver on port" << config["port"].as<std::string>() << std::endl;
                LinkQuad::comm::serial::passive_listen(config["port"].as<std::string>(), receive_command);
                CRAP::comm::listen("/button_press", button_action);
            }
        }


        frequency_t freq(10.0);
        while(ticktock(freq)) {
            check_connectivity();
        }
    }
}
