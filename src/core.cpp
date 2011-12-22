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
#include <string>
#include "yaml-cpp/yaml.h"

/**
 * Variable containing the configuration filename
 */
std::string configuration_filename = "configuration.yaml";
 
/**
 * Main function which is run when the program starts
 * The single command-line argument should be a YAML configuration-file with
 * the 
 */
int main(int argc, const char* argv[])
{
    // Set configuration filename
    if(argc < 2) {
        std::cout << "Using default configuration filename: " << configuration_filename << std::endl;
    } else {
        configuration_filename = argv[1];
    }
    
    // Load CRAP configuration, containing the modules to be loaded
    try {
        //~ YAML::Node modules = YAML::LoadFile(configuration_filename);
        YAML::Node modules = YAML::Load(std::ifstream(configuration_filename));
    }
    catch(YAML::Exception e) {
        std::cout << "No configuration file found." << std::endl;
    }
    
    
    for(YAML::const_iterator module = modules.begin(); module != modules.end();++module) {
        if((*module)["file"]) {
            std::cout << "Adding module: " << (*module)["file"] << std::endl;
        }
    }
}