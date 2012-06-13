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

#ifndef CRAP_MATH_HPP_
#define CRAP_MATH_HPP_

#include <Eigen/Core>
#include "crap/base_types.hpp"
#include <cmath>

#define SQR(x) ((x)*(x))
#define SQRSGN(x) ((x)*std::abs((x)))
#define CUBE(x) ((x)*(x)*(x))
#define POW4(x) ((x)*(x)*(x)*(x))
#define eul2q0(phi,theta,psi) (cos(phi * M_PI/360.0)*cos(theta * M_PI/360.0)*cos(psi * M_PI/360.0) + sin(phi * M_PI/360.0)*sin(theta * M_PI/360.0)*sin(psi * M_PI/360.0))
#define eul2qi(phi,theta,psi) (sin(phi * M_PI/360.0)*cos(theta * M_PI/360.0)*cos(psi * M_PI/360.0) - cos(phi * M_PI/360.0)*sin(theta * M_PI/360.0)*sin(psi * M_PI/360.0))
#define eul2qj(phi,theta,psi) (cos(phi * M_PI/360.0)*sin(theta * M_PI/360.0)*cos(psi * M_PI/360.0) + sin(phi * M_PI/360.0)*cos(theta * M_PI/360.0)*sin(psi * M_PI/360.0))
#define eul2qk(phi,theta,psi) (cos(phi * M_PI/360.0)*cos(theta * M_PI/360.0)*sin(psi * M_PI/360.0) - sin(phi * M_PI/360.0)*sin(theta * M_PI/360.0)*cos(psi * M_PI/360.0))
#define eul2quat(phi,theta,psi) eul2qi(phi,theta,psi),eul2qj(phi,theta,psi),eul2qk(phi,theta,psi),eul2q0(phi,theta,psi)
#define quat2phi(q0,qi,qj,qk)   (std::atan2(2 * (q0*qi + qj*qk), (1-2*(qi*qi + qj*qj))))
#define quat2theta(q0,qi,qj,qk) (std::asin(2*(q0*qj - qk*qi)))
#define quat2psi(q0,qi,qj,qk)   (std::atan2(2 * (q0*qk + qi*qj), (1-2*(qj*qj + qk*qk))))
#define quat2eul(q0,qi,qj,qk)   quat2phi(q0,qi,qj,qk),quat2theta(q0,qi,qj,qk),quat2psi(q0,qi,qj,qk)

namespace CRAP {
    namespace math {
        using namespace Eigen;
        /*!
        * \brief    Computes the fourth-order Runge-Kutta integral approximation of a single step h of a function f from a state x and input u.
        * \param    f   Function to integrate. Should be a function of x and u.
        * \param    x   Starting point of integration
        * \param    u   Control signal. Constant throughout the integration.
        * \param    h   Length of integration
        * \return   New state, with delta added to previous state
        */
        template<int number_of_states, int number_of_controls, typename scalar = base_float_t>
        Matrix<scalar, number_of_states, 1> rk4(
            const Matrix<scalar, number_of_states, 1>&(*f)(const Matrix<scalar, number_of_states, 1>&, const Matrix<scalar, number_of_controls, 1>&),
            const Matrix<scalar, number_of_states, 1>& x,
            const Matrix<scalar, number_of_controls, 1>& u,
            const scalar h
            )
        {
            Matrix<scalar, number_of_states, 1> k1, k2, k3, k4;
            k1 = f(x,u);
            k2 = f(x+0.5*h*k1, u);
            k3 = f(x+0.5*h*k2, u);
            k4 = f(x+h*k3, u);
            return x + h*(k1 + 2*k2 + 2*k3 + k4)*(1/6.);
        }

        /*!
         * \brief   Calculate the linear array index of a two dimensional matrix
         * \param   x   Matrix row position
         * \param   y   Matrix column position
         * \return  Linear array index
         */
        template<unsigned START, unsigned LEADING_DIM>
        inline unsigned index(int x,int y) {
            return x + START + LEADING_DIM*y;
        }

        template<class T>
        inline T floor0(T value) {
            if(value < 0.0)
                return std::ceil( value );
            else
                return std::floor( value );
        }

        template<class T>
        inline T round(T value) {
            return std::floor( value + 0.5 );
        }

        template<class T>
        inline T nearest_angle(T a, T b) {
            return a + round<T>((b-a)/(2*M_PI))*2*M_PI;
        }

        template<class T>
        inline T nearest_angle_between(T a, T b) {
            return round<T>((b-a)/(2*M_PI))*2*M_PI;
        }

        template <typename T> int sign(T val) {
            return (val > T(0)) - (val < T(0));
        }
    }
}

#endif
