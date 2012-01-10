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

#include <iostream>
#include <fstream>
#include <dlfcn.h>
#include <string>
#include <algorithm>
#include <map>
#include "yaml-cpp/yaml.h"

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/timer.hpp>


namespace CRAP {
    /// Typedefs for module maps
    typedef std::map<std::string, void*> module_map;
    typedef std::map<std::string, boost::shared_ptr<boost::thread> > thread_map;
    typedef void(*configuration_function)(YAML::Node&);
    typedef std::map<std::string, configuration_function> configuration_map;

    /**
     * Global starting time
     */
    boost::timer starting_time;


    /**
     * Variable containing YAML-data with system configuration
     */
    YAML::Node config;

    /**
     * Map of all nodes' configurations
     */
    std::map<std::string, YAML::Node> node_config;

    /**
     * Map of all loaded modules
     */
    module_map modules;

    /**
     * Map of the modules' threads
     */
    thread_map threads;

    /**
     * Links to all the configuration functions of the modules that have them,
     * and a initializing configuration specified in the module configuration YAML-file.
     */
    configuration_map configuration_functions;


    void reconfigure(std::string);
    void register_module(const std::string, const std::string, const bool = false);
    void register_module_yaml(const YAML::Node);
    void close_thread(const std::string);
    void unregister_module(const std::string);
    void unregister_all_modules();
    void setup(std::string);

    /**
     * Call the configuration function of the module
     * \param name Name of the module to be reconfigured
     */
    void reconfigure(std::string name) {
        configuration_functions[name](node_config[name]);
    }

    /**
     * Register a module and add it to the list of added modules.
     * Then execute the 'run' method of the module as a stand-alone thread
     * \param name Name of the module to be added. This must be unique for all module instances.
     * \param file Path to the file of the module
     */
    void register_module(const std::string name, const std::string file, bool configure) {
        void* dlib;
        std::string module_path;

        module_path = config["module_root"].as<std::string>() + file; //config["module_root"].as<std::string>("")
        std::cout << "Adding module: " << module_path << std::endl;
        dlib = dlopen(module_path.c_str(), RTLD_LAZY);

        if(dlib == NULL) {
            std::cerr << "Failed to link module: " << module_path << "(" << dlerror() << ")" << std::endl;
            return; //FIXME: Throw exception?
        }

        modules[name] = dlib;

        if(configure) {
            void* cfn = dlsym(modules[name], "configure");
            if(cfn == NULL) {
                std::cerr << "No function 'configure' was found, but configuration was given" << std::endl;
            } else {
                configuration_functions[name] = (configuration_function)cfn;
                reconfigure(name);
            }
        }

        void* rfn = dlsym(dlib, "run");
        threads[name] = boost::shared_ptr<boost::thread>(new boost::thread((void(*)())rfn));
    }

    /**
     * Register a module and add it to the list of added modules.
     * Then execute the 'run' method of the module as a stand-alone thread
     * \param module Module configuration as input from a YAML configuraton file
     */
    void register_module_yaml(const YAML::Node module) {
        assert(module["file"]);
        assert(module["name"]);

        std::string name = module["name"].as<std::string>();

        bool configure = false;
        if(module["configuration"]) {
            if(module["configuration"].IsScalar()) {
                //~ node_config[name] = YAML::LoadFile(module["configuration"]);
            } else {
                node_config[name] = module["configuration"];
            }
            configure = true;
        }
        register_module(name, module["file"].as<std::string>(), configure);
    }

    /**
     * Join a thread and remove it from the list of running threads
     * \param key Name of the module whos thread to close down.
     */
    void close_thread(const std::string key) {
        threads[key]->interrupt();
        if(!threads[key]->timed_join(boost::posix_time::milliseconds(5000))) {
            std::cerr << "Failed to join thread within five seconds. Dropping thread" << std::endl;
        }
        threads.erase(key);
    }

    /**
     * Unregister a single module. Closes the dynamic link bindings and kills
     * any running threads.
     * \param key Name of the module to unregister
     */
    void unregister_module(const std::string key) {
        close_thread(key);
        dlclose(modules[key]);
        modules.erase(key);
    }

    /**
     * Close ALL THE THREADS
     */
    void unregister_all_modules() {
        while(!modules.empty()) {
            unregister_module(modules.begin()->first);
        }
    }

    /**
     * Join all threads to exit
     */
    inline void join() {
        unregister_all_modules();
    }

    inline void spin() {
        while(true) {
            boost::this_thread::yield();
        }
    }

    void go() {
        boost::thread(spin).join();
    }

    /**
     * Initalize the system by loading the modules specified by the current
     * configuration file
     * \param configuration_filename Filename of the current YAML configuration file
     */
    void setup(std::string configuration_filename) {
        std::ifstream configuration_file;
        configuration_file.exceptions ( std::ifstream::failbit | std::ifstream::badbit );
        try {
            //~ modules = YAML::LoadFile(configuration_filename);
            configuration_file.open(configuration_filename.c_str());
            config = YAML::Load(configuration_file);

            // Link the modules
            std::for_each(config["modules"].begin(), config["modules"].end(), register_module_yaml);
        }
        catch(std::ifstream::failure e) {
            std::cerr << "Could not load configuration file: " << e.what() << std::endl;
        }
        catch(YAML::Exception e) {
            std::cerr << "Error parsing the configuration file: " << e.what() << std::endl;
        }
    }
}

/**
 * Main function which is run when the program starts
 * The single command-line argument should be a YAML configuration-file with
 * the
 */
int main(int argc, char* argv[])
{
    /**
     * Variable containing the configuration filename
     */
    std::string configuration_filename = "configuration.yaml";

    // Set configuration filename
    if(argc < 2) {
        std::cout << "Using default configuration filename: " << configuration_filename << std::endl;
    } else {
        configuration_filename = argv[1];
    }


    // Load CRAP configuration
    CRAP::setup(configuration_filename);
    std::cout << "go!!" << std::endl;
    CRAP::spin();
    std::cout << "went!" << std::endl;
}

