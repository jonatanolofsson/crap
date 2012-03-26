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

#include "camera.hpp"

namespace CPGL {
    Camera::Camera(YAML::Node& c, BaseElement* p) : core::BaseElement(c, p) {
        look_at((Vector3f() << 0,20,2).finished(), (Vector3f() << 0,20,0).finished(), Vector3f::UnitY());
    }

    void Camera::rotation_from_dxdy(int dx, int dy) {
        static const float fiscale = 100.0;

        base =
            (AngleAxis<float>(((float)dy)/fiscale, Vector3f::UnitX()))*
            (AngleAxis<float>(((float)dx)/fiscale, base.linear()*Vector3f::UnitY()))*
            base;
    }

    Vector3f Camera::position() {
        return -base.linear().transpose() * base.translation();
    }
}

extern "C" {
    using namespace CPGL::core;
    BaseElement* factory(YAML::Node& c, BaseElement* p) {
        return dynamic_cast<BaseElement*>(new CPGL::Camera(c,p));
    }
}
