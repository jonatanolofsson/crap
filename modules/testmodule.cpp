#include <iostream>

extern "C" {
    void run() {
        std::cout << "This is from inside the testmodule" << std::endl;
    }
}
