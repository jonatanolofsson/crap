/*
 * License: Expat
 * Copyright (C) 2011 by file author
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

/*!
 * \file    PID.hpp
 *
 * \author  Jonatan Olofsson [jonatan.olofsson@gmail.com]
 * \date    2011
 * \brief   Simple PID-controller with anti-windup
 *
 * This file contains a simple implementation of a discret PID controller on
 * standard form. Anti-windup is implemented using control-signal limits.
 */

#ifndef PID_hpp_
#define PID_hpp_

#include <Eigen/Core>

namespace control {
    using namespace Eigen;

    /*!
     * \class   PID
     * \author  Jonatan Olofsson    [jonatan.olofsson@gmail.com]
     * \date    2011
     * \brief   Simple PID controller with anti-windup
     *
     * A good description of a the parameters on the standard form can be found on Wikipedia:
     * http://en.wikipedia.org/wiki/PID_controller#Ideal_versus_standard_PID_form
     */
    template<int number_of_states>
    class PID  {
        typedef float gain_t;
        typedef float dt_t;
        typedef Matrix<gain_t, number_of_states, number_of_states> gain;
        typedef Matrix<gain_t, number_of_states, 1> controlsignal;
        typedef Matrix<gain_t, number_of_states, 1> states;
        public:
            gain K;   ///< Proportional gain
            gain Ti_inv;  ///< Integral time
            gain Td;  ///< Derivative time.
            dt_t Ts; ///< Sampling time
            controlsignal umin, umax; ///< Control-signal limits

        private:
            states I;   ///< Integration variable for storing the sum of all previous errors
            states ep;  ///< Store the previous error
            controlsignal u;   ///< Current output from controller

        public:
            /*!
             * \brief   Construct a PID controller
             * \param   K   Proportional gain of PID controller
             * \param   Ti_inv  Integral time of PID controller
             * \param   Td  Derivative time of PID controller
             * \param   umin    Lower control-signal limit
             * \param   umax    Upper control-signal limit
             */
            PID(gain K,  gain Ti_inv, gain Td, dt_t Ts, controlsignal umin, controlsignal umax) {
                PID::K = K;
                PID::Ti_inv = Ti_inv;
                PID::Td = Td;
                PID::umin = umin;
                PID::umax = umax;
            }

            /*!
             * \brief   Construct a PID controller
             */
            PID(){
                K.setZero();
                Ti_inv.setZero();
                Td.setZero();
                umin.setZero();
                umax.setIdentity();
            }

            /*!
             * \brief   Calculate a new control-signal
             *
             * This steps the controller one step and alters the internal
             * state of the controller.
             * \param   e   Current error
             * \return  New control-signal
             */
            controlsignal control_signal(states e) {
                states Iadd = K*Ti_inv*Ts*e;
                u = K*e + I + Iadd + K*Td/Ts*(e - ep);
                for(int i=0; i<number_of_states; ++i) {
                    gain_t a = u(i)-umin(i);
                    if(a < 0) { // Signal lower than minimum
                        u(i) = umin(i);
                        I(i) += Iadd(i)-a;
                    } else {
                        gain_t b = u(i)-umax(i);
                        if(b > 0) { // Signal larger than maximum
                            u(i) = umax(i);
                            I(i) += Iadd(i)-b;
                        } else { // Signal is actually in range
                            I(i) += Iadd(i);
                        }
                    }
                }
                ep = e;
                return u;
            }

            /*!
             * \brief   Calculate a new control-signal
             *
             * This steps the controller one step and alters the internal
             * state of the controller.
             * \param   y   Current output
             * \param   r   Current reference
             * \return  New control-signal
             */
            controlsignal control_signal(states y, states r) {
                return control_signal(r-y);
            }

            /*!
             * \brief   Get the latest calculated control-signal
             * \return  Latest calculated control-signal
             */
            controlsignal current_control_signal() {
                return u;
            }

            /*!
             * \brief   Get current integral parts
             * \return  Latest calculated control-signal
             */
            states current_integrals() {
                return I;
            }

            void init_imc(int state, gain_t Kp, gain_t T, gain_t L, gain_t Tc) {
                K(state, state) = (0.5*L + T)/(Kp * (Tc + L));
                Ti_inv(state, state) = 1/(0.5*L + T);
                Td(state, state) = L*T/(L + 2*T);
            }


    };
}

#endif
