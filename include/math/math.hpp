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
        template<int number_of_states, int number_of_controls>
        Matrix<double, number_of_states, 1> rk4(
            Matrix<double, number_of_states, 1>&(*f)(const Matrix<double, number_of_states, 1>&, const Matrix<double, number_of_controls, 1>&),
            const Matrix<double, number_of_states, 1>& x,
            const Matrix<double, number_of_controls, 1>& u,
            const double h
            )
        {
            Matrix<double, number_of_states, 1> k1, k2, k3, k4;
            k1 = f(x,u);
            k2 = f(x+0.5*h*k1, u);
            k3 = f(x+0.5*h*k2, u);
            k4 = f(x+h*k3, u);
            return x + h*(k1 + 2*k2 + 2*k3 + k4)*(1/6.);
        }

        template<class T>
        inline T floor0(T value)
        {
            if(value < 0.0)
                return std::ceil( value );
            else
                return std::floor( value );
        }

        template<class T>
        inline T round(T value)
        {
            return std::floor( value + 0.5 );
        }

        template<class T>
        inline T nearest_angle(T a, T b)
        {
            return a + round<T>((b-a)/(2*M_PI))*2*M_PI;
        }
    }
}

#endif
