#include <iostream>
#include "crap/module.hpp"

extern "C" {
    void run() {
        std::cout << "This is from inside the testmodule" << std::endl;
        LOG_INFO << "This is information from the testmodule";
    }
}
