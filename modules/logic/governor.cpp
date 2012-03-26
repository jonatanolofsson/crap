#include <map>
#include <boost/thread/mutex.hpp>
#include <assert.h>
#include "modules/logic/governor.hpp"
#include <dlfcn.h>
#include "crap/state_engine.hpp"
#include <iostream>
#include <yaml-cpp/yaml.h>

namespace CRAP {
    namespace logic {
        namespace governor {
            typedef void(*configuration_function)(YAML::Node&);
            typedef std::pair<void*, void*> modepair_t;
            typedef std::map<std::string, modepair_t> modemap_t;
            typedef std::map<std::string, void*> modehandlemap_t;
            modemap_t modes;
            modehandlemap_t mode_handles;
            state_engine_t state_machine;

            std::string current_mode;
            YAML::Node config;


            /**
             * Map of all nodes' configurations
             */
            std::map<std::string, YAML::Node> mode_config;


            void configure(YAML::Node& c) {
                config = c;
            }

            void load_and_configure(const std::string mode) {
                mode_config[mode] = YAML::LoadFile(config["modes"]["directory"].as<std::string>("") + mode + config["modes"]["configuration_filename"].as<std::string>("configuration.yaml"));
                if(mode_config[mode]) {
                    ((configuration_function)modes[mode].second)(mode_config[mode]);
                }
            }

            bool load_mode(const std::string mode, const std::string fn) {
                std::cout << "Load mode." << std::endl;
                std::cout << "Loading flightmode: " << mode << std::endl;
                if(modes.find(mode) != modes.end()) return true;
                void* m;
                m = dlopen((config["modes"]["directory"].as<std::string>("") + mode + config["modes"]["extension"].as<std::string>(".so")).c_str(), RTLD_LAZY);
                if(m) {
                    void* f;
                    f = dlsym(m, fn.c_str());
                    if(f) {
                        mode_handles[mode] = m;
                        std::cout << "Loaded flightmode: " << mode << std::endl;

                        void* fc;
                        fc = dlsym(m, "configure");
                        if(fc) {
                            std::cout << "Configuration function found" << std::endl;
                            modes[mode] = modepair_t(f, fc);
                            load_and_configure(mode);
                        } else {
                            std::cout << "No configuration function found" << std::endl;
                            modes[mode] = modepair_t(f, NULL);
                        }
                        return true;
                    }
                    else {
                        std::cout << "Function `" << fn << "` not found in loaded file: " << mode << std::endl;
                        dlclose(m);
                    }
                }
                std::cout << "Flightmode loading failed: " << mode << std::endl;
                return false;
            }


            void switch_mode(const std::string& mode) {
                if(mode == current_mode) return;
                if(load_mode(mode, mode)) {
                    state_machine.next(modes[mode].first);
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
