#include "yaml-cpp/yaml.h"
#include "spnav_utils.hpp"
#include <iostream>
#include "linkquad/serial_communication.hpp"
#include "modules/baselink/comm_messages.hpp"


YAML::Node config;

void send_message(spnav::event& sev) {
    CRAP::comm_messages::comm_message message;
    if(sev.type == SPNAV_EVENT_MOTION) {
        BIT_SET(message.params_8i_0, CRAP::comm_messages::TRANSLATION_AND_ROTATION);
        message.params_32f_0 = sev.motion.z     * config["scale"]["x"].as<float>(1.0);
        message.params_32f_1 = sev.motion.x     * config["scale"]["y"].as<float>(1.0);
        message.params_32f_2 = -sev.motion.y    * config["scale"]["z"].as<float>(1.0);

        message.params_32f_3 = sev.motion.rz     * config["scale"]["rx"].as<float>(1.0);
        message.params_32f_4 = sev.motion.rx     * config["scale"]["ry"].as<float>(1.0);
        message.params_32f_5 = -sev.motion.ry    * config["scale"]["rz"].as<float>(1.0);
    } else {    /* SPNAV_EVENT_BUTTON */
        if(sev.button.press) {
            BIT_SET(message.params_8i_0, CRAP::comm_messages::BUTTON_PRESS);
        } else {
            BIT_SET(message.params_8i_0, CRAP::comm_messages::BUTTON_RELEASE);
        }
        message.params_8i_1 = sev.button.bnum;
    }
    LinkQuad::comm::serial::send(config["port"].as<std::string>(), message);
}

int main(int argc, char* argv[])
{
    if(argc < 2) {
        std::cerr << "No configurationfile given" << std::endl;
        return 1;
    }
    config = YAML::LoadFile(argv[1]);
    spnav::readloop(send_message);
    return 0;
}
