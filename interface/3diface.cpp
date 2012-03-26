#include <Eigen/Core>
#include <Eigen/Geometry>

#include "yaml-cpp/yaml.h"
#include "spnav_utils.hpp"
#include <iostream>
#include "linkquad/serial_communication.hpp"
#include "modules/baselink/comm_messages.hpp"


#include "modules/model/model.hpp"
#include "cpgl/cpgl.hpp"
#include "cpgl/Window.hpp"
#include "elements/flyer/flyer.hpp"
#include "elements/camera/camera.hpp"
#include <cmath>
#include <string.h>
//~ #include "elements/flyer/flyer.cpp"

int argc; char** argv;

namespace visualizer {
    using namespace Eigen;
    using namespace CRAP::observer;
    using namespace CRAP::observer::model;
    YAML::Node config;

    void send_message(spnav::event& sev) {
        float params_32f_x = 0;
        float params_32f_y = 0;
        CRAP::comm_messages::command_message message;
        memset(&message, 0, sizeof(message));
        if(sev.type == SPNAV_EVENT_MOTION) {
            BIT_SET(message.params_8i_0, CRAP::comm_messages::TRANSLATION_AND_ROTATION);
            message.params_32f_0 = (sev.motion.z     * config["output_range"]["x"].as<float>(1.0)) / config["input_range"][2][sev.motion.z>0].as<float>(1.0);
            message.params_32f_1 = (sev.motion.x     * config["output_range"]["y"].as<float>(1.0)) / config["input_range"][0][sev.motion.x>0].as<float>(1.0);
            message.params_32f_2 = (-sev.motion.y    * config["output_range"]["z"].as<float>(1.0)) / config["input_range"][1][sev.motion.y>0].as<float>(1.0);

            params_32f_x = (sev.motion.rz    * config["output_range"]["rx"].as<float>(1.0)) / config["input_range"][5][sev.motion.rz>0].as<float>(1.0);
            params_32f_y = (sev.motion.rx    * config["output_range"]["ry"].as<float>(1.0)) / config["input_range"][3][sev.motion.rx>0].as<float>(1.0);
            message.params_32f_3 = (-sev.motion.ry   * config["output_range"]["rz"].as<float>(1.0)) / config["input_range"][4][sev.motion.ry>0].as<float>(1.0);
        } else {    /* SPNAV_EVENT_BUTTON */
            if(sev.button.press) {
                BIT_SET(message.params_8i_0, CRAP::comm_messages::BUTTON_PRESS);
            } else {
                BIT_SET(message.params_8i_0, CRAP::comm_messages::BUTTON_RELEASE);
            }
            message.params_8i_1 = sev.button.bnum;
        }
        if(config["port"]) {
            //~ std::cout << "Sending message on " << config["port"].as<std::string>() << std::endl;
            //~ std::cout << "T(" << message.params_32f_0 << ", " << message.params_32f_1 << ", " << message.params_32f_2 << ")"
                        //~ << "\tR(" << params_32f_x << ", " << params_32f_y << ", " << message.params_32f_3 << ")" << std::endl;
            LinkQuad::comm::serial::send(config["port"].as<std::string>(), message);
        } else {
            std::cout << "T(" << message.params_32f_0 << ", " << message.params_32f_1 << ", " << message.params_32f_2 << ")"
                        << "\tR(" << params_32f_x << ", " << params_32f_y << ", " << message.params_32f_3 << ")" << std::endl;
        }
    }


    CPGL::window_t win;
    CPGL::Flyer* flyer;
    CPGL::Camera* camera;
    //~ CPGL::Gauges* gauges;

    void set_window(CPGL::window_t window) {
        std::cout << "Setting window" << std::endl;
        win = window;
        flyer = dynamic_cast<CPGL::Flyer*>(win->get("flyer"));
        camera = dynamic_cast<CPGL::Camera*>(win->get("camera"));
        //~ std::cout << "Flyer: " << win->get("flyer") << std::endl;
    }

    Matrix3f ned2world((Matrix3f() <<
        0,1,0,
        0,0,-1,
        -1,0,0
    ).finished());

    void update_state(const CRAP::comm_messages::state_message d) {
        if(flyer == NULL) return;
        Vector3f p; p << d.params_32f_0, d.params_32f_1, d.params_32f_2;
        //~ std::cout << "P: " << p.transpose() << "; Pworld: " << (ned2world*p).transpose() << std::endl;
        Quaternion<float> q(
            d.params_32f_3,
            d.params_32f_4,
            d.params_32f_5,
            d.params_32f_6
        );

        flyer->base.translation() = ned2world * p;
        flyer->base.linear() = ned2world * q.toRotationMatrix() * ned2world.transpose();
        
        Vector3f cpos(flyer->base.translation());
        cpos.z() += 3.0;
        camera->look_at(cpos, flyer->base.translation(), Vector3f::UnitY());
    }

    //~ void update_control(const reference_vector& u) {
        //~ gauges->update(u);
    //~ }

    void run(YAML::Node c) {
        config = c;
        std::cout << "Init visualization" << std::endl;
        spnav::listen(send_message);
        flyer = NULL;
        CPGL::init(argc, argv, config, set_window);
        if(config["port"]) {
            std::cout << "Listening for new states on " << config["port"].as<std::string>() << std::endl;
            LinkQuad::comm::serial::passive_listen(config["port"].as<std::string>(), update_state);
        }
        float t = 0;
        CPGL::wait();
    }
}

int main(int argc_, char* argv_[])
{
    argc = argc_; argv = argv_;
    if(argc < 2) {
        std::cerr << "No configurationfile given" << std::endl;
        return 1;
    }

    visualizer::run(YAML::LoadFile(argv[1]));

    return 0;
}
