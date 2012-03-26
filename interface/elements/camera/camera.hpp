/**
 * Copyright 2011, 2012 Jonatan Olofsson
 *
 * This file is part of C++ GL Framework (CPGL).
 *
 * CPGL is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CPGL is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with CPGL.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CPGL_CAMERA_HPP_
#define CPGL_CAMERA_HPP_

#include "cpgl/cpgl.hpp"
#include "cpgl/BaseElement.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace CPGL {
    using namespace core;
    using namespace tools;
    class Camera : public BaseElement {
        private:
            GLuint program;
            Model* object;
            int mouse_click[2];

        public:
            Camera(YAML::Node& c, BaseElement* p);

            void draw(){}

            Vector3f position();
            void rotation_from_dxdy(int dx, int dy);


			bool keyboard(unsigned char key, int, int) {
				if(key == 'l') {
					rotation_from_dxdy(1,0);
					return false;
				} else if(key == 'j') {
					rotation_from_dxdy(-1,0);
					return false;
				} else if(key == 'i') {
					rotation_from_dxdy(0,1);
					return false;
				} else if(key == 'k') {
					rotation_from_dxdy(0,-1);
					return false;
				}

				Vector3f dposition; dposition.setZero();
				if(key == 'w') {
					dposition = -Vector3f::UnitZ();
				} else if(key == 's') {
					dposition = Vector3f::UnitZ();
				} else if(key == 'd') {
					dposition = Vector3f::UnitX();
				} else if(key == 'a') {
					dposition = -Vector3f::UnitX();
				} else if(key == 'e') {
					dposition = Vector3f::UnitY();
				} else if(key == 'q') {
					dposition = -Vector3f::UnitY();
				} else return false;

				base.translation() -= dposition * config["speed_factor"].as<float>(1.0) * 0.1;
				return false;
			}


			bool motion(int x, int y) {
				rotation_from_dxdy(x-mouse_click[0], y-mouse_click[1]);

				mouse_click[0] = x;
				mouse_click[1] = y;
				return false;
			}


			bool mouse(int button, int state, int x, int y) {
				mouse_click[0] = x;
				mouse_click[1] = y;
				return false;
			}
    };
}

#endif
