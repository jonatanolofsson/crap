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

#ifndef CRAP_MATH_CONTROL_LINEAR_MODEL_HPP_
#define CRAP_MATH_CONTROL_LINEAR_MODEL_HPP_

#include <Eigen/Core>
#include <Eigen/LU>
namespace CRAP {
    namespace control {
        using namespace Eigen;

        template<int number_of_states, int number_of_controls, typename scalar = base_float_t>
        struct linear_model {
            typedef Matrix<scalar, number_of_states, number_of_states> state_matrix;
            typedef Matrix<scalar, number_of_states, number_of_states> state_weight_matrix;
            typedef Matrix<scalar, number_of_states, number_of_controls> control_matrix;
            typedef Matrix<scalar, number_of_controls, number_of_controls> control_weight_matrix;
            typedef Matrix<scalar, number_of_controls, number_of_states> feedforward_selection_matrix;

            char DICO; ///< 'D'iscrete or 'C'continous (default) model

            state_matrix A;
            control_matrix B;
            feedforward_selection_matrix M;

            state_weight_matrix Q; ///< State weights for LQ-controller
            control_weight_matrix R; ///< Signal weights for LQ-controller
            linear_model() : DICO('C'){A.setZero(); B.setZero(); Q.setIdentity(); R.setIdentity(); M.setZero();}
        };


        template<int number_of_states, int number_of_controls, int starting_state = 0, int starting_control = 0, typename scalar = base_float_t, typename Derivedf, typename Derivedx, typename Derivedu, typename DerivedA, typename DerivedB>
        void system_jacobian(
            Derivedf f,
            const Derivedx& x0,
            const Derivedu& u0,
            DerivedA& A)
        {
            static const scalar delta = 1e-3;
            static const scalar div = 0.5/delta;

            Derivedx x(x0);
            for(unsigned int i = starting_state; i < number_of_states + starting_state; ++i) {
                x[i] += delta;
                A.col(i-starting_state) = div*f(x,u0).template segment<number_of_states>(starting_state);
                x[i] = x0[i]-delta;
                A.col(i-starting_state) -= div*f(x,u0).template segment<number_of_states>(starting_state);
                x[i] = x0[i];
            }
        }

        template<int number_of_states, int number_of_controls, int starting_state = 0, int starting_control = 0, typename scalar = base_float_t, typename Derivedf, typename Derivedx, typename Derivedu, typename DerivedB>
        void control_jacobian(
            Derivedf f,
            const Derivedx& x0,
            const Derivedu& u0,
            DerivedB& B)
        {
            static const scalar delta = 1e-3;
            static const scalar div = 0.5/delta;

            Derivedu u(u0);
            for(unsigned int i = starting_control; i < number_of_controls+starting_control; ++i) {
                u[i] += delta;
                Block<DerivedB, number_of_states, 1> Bcol = B.template block<number_of_states, 1>(starting_state, i-starting_control);
                Bcol = div*f(x0,u).template segment<number_of_states>(starting_state);
                u[i] = u0[i] - delta;
                Bcol -= div*f(x0,u).template segment<number_of_states>(starting_state);
                u[i] = u0[i];
            }
        }


        template<int number_of_states, int number_of_controls, int starting_state = 0, int starting_control = 0, typename scalar = base_float_t, typename Derivedf, typename Derivedx, typename Derivedu, typename DerivedA, typename DerivedB>
        void jacobians(
            Derivedf f,
            const Derivedx& x0,
            const Derivedu& u0,
            DerivedA& A,
            DerivedB& B)
        {
            static const scalar delta = 1e-3;
            static const scalar div = 0.5/delta;

            Derivedx x(x0);
            for(unsigned int i = starting_state; i < number_of_states + starting_state; ++i) {
                x[i] += delta;
                A.col(i-starting_state) = div*f(x,u0).template segment<number_of_states>(starting_state);
                x[i] = x0[i]-delta;
                A.col(i-starting_state) -= div*f(x,u0).template segment<number_of_states>(starting_state);
                x[i] = x0[i];
            }

            Derivedu u(u0);
            for(unsigned int i = starting_control; i < number_of_controls+starting_control; ++i) {
                u[i] += delta;
                B.col(i-starting_control) = div*f(x0,u).template segment<number_of_states>(starting_state);
                u[i] = u0[i] - delta;
                B.col(i-starting_control) -= div*f(x0,u).template segment<number_of_states>(starting_state);
                u[i] = u0[i];
            }
        }
    }
}

#endif
