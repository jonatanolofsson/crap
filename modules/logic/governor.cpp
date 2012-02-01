#include <map>
#include <boost/thread/mutex.hpp>
#include <assert.h>
#include "modules/logic/governor.hpp"
#include <dlfcn.h>
#include "crap/state_engine.hpp"

namespace CRAP {
    namespace logic {
        namespace governor {
            typedef std::map<std::string, void*> modemap_t;
            modemap_t modes;
            modemap_t mode_handles;
            state_engine_t state_machine;

            std::string current_mode;
            YAML::Node config;


            void configure(YAML::Node& c) {
                config = c;
            }

            bool load_mode(const std::string mode) {
                if(modes.find(mode) != modes.end()) return true;
                void* m;
                m = dlopen((config["modes"]["directory"].as<std::string>("") + mode + config["modes"]["extension"].as<std::string>(".so")).c_str(), RTLD_LAZY);
                if(m) {
                    void* f;
                    f = dlsym(m, mode.c_str());
                    if(f) {
                        modes[mode] = f;
                        mode_handles[mode] = m;
                        std::cout << "Loaded flightmode: " << mode << std::endl;
                        return true;
                    }
                    else {
                        std::cout << "Function not found in loaded file: " << mode << std::endl;
                        dlclose(m);
                    }
                }
                std::cout << "Flightmode loading failed: " << mode << std::endl;
                return false;
            }


            void switch_mode(const std::string& mode) {
                if(mode == current_mode) return;
                if(load_mode(mode)) {
                    state_machine.next(modes[mode]);
                    current_mode = mode;
                }
            }

            void tick() {
                //if(battery level < battery threshold) switch_mode("landing");

                state_machine();
            }
        }
    }
}
