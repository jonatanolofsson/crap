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

#ifndef CRAP_CONTROLLER_MODEL_HPP_
#define CRAP_CONTROLLER_MODEL_HPP_
#include <Eigen/Core>
#include "crap/base_types.hpp"

namespace CRAP {
    namespace controller {
        using namespace Eigen;
        namespace model {
            const int number_of_states              = 12;
            const int number_of_linearized_states   = 12;
            const int number_of_controls            = 4;
            const int number_of_references          = 4;

            namespace state {
                const int roll = 10;
                const int pitch = 11;
            }
        }
        namespace detail {
            typedef double scalar;
            typedef Matrix<scalar, model::number_of_controls, 1> control_vector;
            typedef Matrix<scalar, model::number_of_references, 1> reference_vector;
        }

        typedef Matrix<base_float_t, model::number_of_controls, 1> control_vector;
        typedef Matrix<base_float_t, model::number_of_references, 1> reference_vector;
        typedef control_vector(*control_signal_fn)();
    }
}

#endif
