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
 * \file    LQ.hpp
 *
 * \author  Jonatan Olofsson    [jonatan.olofsson@gmail.com]
 * \date    2011
 * \brief   Templated implementation of Linear Quadratic Control
 *
 * The file contains two related implementations of the Linear Quadratic Regulator.
 * The standard LQ implementation is a straight-forward implementation
 * of the LQ controller which is fed with a linear model providing the
 * nescessary matrices: state description A, control signal description B,
 * control selecting matrix M, state cost regulation matrix Q and
 * control signal regulation matrix R.
 *
 * The second implementation, named Dynamic LQ (DLQ), does the same thing
 * as the static LQ, with the exception that the model is re-linearized
 * each iteration. This can be used for non-linear cases when computations
 * are cheap.
 *
 * Both classes are, after setup, interfaced with a commonly named function
 * \em control_signal , which can be called with or without a reference to follow.
 */

#ifndef LQ_hpp_
#define LQ_hpp_

#include "math/control/linear_model.hpp"
#include "math/control/slicot.hpp"
#include <Eigen/Core>
#include <Eigen/QR>

//~ #include <iostream>

namespace CRAP {
    namespace control {
        using namespace Eigen;

        /*!
         * \class   LQ
         * \author  Jonatan Olofsson    [jonatan.olofsson@gmail.com]
         * \date    2011
         * \brief   A Linear Quadratic Regulator
         *
         * A LQR is used to calculate an optimal control signal to bring a linear
         * system to a desired state, given the system model and an optional reference.
         * The control signal is optimal in the sense that it minimizes a cost
         * function parametrized by the cost matrices of the model, where a cost
         * is defined for each combination of states and control signals respectively.
         */
        template<int number_of_states, int number_of_controls>
        class LQ {
            public:
                typedef linear_model<number_of_states, number_of_controls> controller_model; ///< Define the linear model type
                typedef Matrix<double, number_of_controls, 1> control_vector; ///< A control vector contains information about the controller output
                typedef Matrix<double, number_of_states, number_of_controls> control_matrix; ///< A control matrix relates the controller output to the model response
                typedef Matrix<double, number_of_states, 1> state_vector; ///< A state vector describes the state in which the system is (currently) in
                typedef Matrix<double, number_of_states, number_of_states> state_matrix; ///< A state (propagation) matrix describes how the model state evolves in time, given no control
                typedef Matrix<double, number_of_controls, number_of_states> feedback_matrix; ///< A feedback matrix is used to calculate the optimal control signal, given a current state fo the system, to bring the system to a zero-state
                typedef Matrix<double, number_of_controls, number_of_states> control_selection_matrix; ///< A control selection matrix is used to select the states of the system that are relevant for control.
                typedef Matrix<double, number_of_controls, number_of_controls> feedforward_matrix; ///< A feedforward matrix is used to calculate the gain needed on the reference signal to optimally follow the reference
                typedef Matrix<double, number_of_controls, 1> reference_vector; ///< A reference vector contains the desired state of the controlled signals.

            protected:
                control_vector u; ///< u is used for storing the computed control signal
                feedback_matrix L; ///< Feedback matrix
                ColPivHouseholderQR<feedforward_matrix> Lr_inv; ///< For numerical reasons, the inverse of the feedforward matrix Lr is stored

            public:
                controller_model m; ///< Internal storage of linear model

                /*!
                 * \brief   Update the controller with a new linear model.
                 *
                 * Recalculates the nescessary internally used matrices.
                 * \param   m   Linear model of the same size as the previous
                 */
                void set_model(const controller_model &m) {
                    LQ::m = m;
                    calculate_control_matrices();
                }
                /*!
                 * \brief   Recalculates the nescessary internally used matrices.
                 */
                void refresh() {
                    calculate_control_matrices();
                }

                /*!
                 * \author  Jonatan Olofsson
                 * \date    2011
                 * \brief   Default constructor
                 */
                LQ() {
                    u.setZero();
                }

                /*!
                 * \brief   Initiate a new filter using a given model
                 *
                 * Calculates the nescessary internally used matrices.
                 * \param   m   Linear model of the same size as the filter was constructed for
                 */
                LQ(const controller_model &m) {
                    u.setZero();
                    set_model(m);
                }

                /*!
                 * \brief   Calculate the control signal optimal for following reference
                 * \param   x   Current state of system
                 * \param   r   Reference to bring the system to
                 * \return  Optimal control signal for following reference, given cost matrices of model
                 */
                const control_vector& control_signal(const state_vector& x, const reference_vector& r) {
                    /// [2] eq. 9.11: u = Lr*r - L*x
                    u = Lr_inv.solve(r) - L*x;
                    return u;
                }

                /*!
                 * \brief   Calculate the control signal optimal for bringing the system to the origin
                 * \param   x   Current state of system
                 * \return  Optimal control signal, given cost matrices of model
                 */
                const control_vector& control_signal(const state_vector& x) {
                    /// [2] eq. 9.11: u = L*x, r = 0
                    u = L*x;
                    return u;
                }

                /*!
                 * \brief   Get the current output signal
                 * \return  The last calculated control signal
                 */
                const control_vector& current_control_signal() {
                    return u;
                }

            protected:
                /*!
                 * \brief   Calculate the matrices L and Lr^-1
                 *
                 * The equations used in this function are formulated in
                 * [Glad & Ljung, Reglerteori (Studentlitteratur, 2003)]
                 *
                 * For numerical reasons, a QR decomposition of Lr is stored
                 * and solved for each incoming control signal when needed.
                 */
                void calculate_control_matrices() {
                    state_matrix S; /// S is the PSD solution to the discrete Riccati equation
                    are<number_of_states, number_of_controls>(m, S);

                    if(m.DICO == 'C') { // Continous system
                        //FIXME: Verify
                        ///
                        L = m.R.householderQr().solve((m.B.transpose()*S).eval());

                        ///
                        Lr_inv.compute((m.M*((m.B*L - m.A).householderQr().solve(m.B.eval()))));
                    } else if(m.DICO == 'D') { // Discrete system
                        /// [2] eq. 9.37a: L = (B'SB + Q2)\B'SA
                        L = (m.B.transpose()*S*m.B + m.R).householderQr().solve((m.B.transpose()*S*m.A).eval());

                        /// [2] eq. 9.38½: Lr*r = ( M * (I + BL - A)\B )\r
                        Lr_inv.compute((m.M*((state_matrix::Identity() + m.B*L - m.A).householderQr().solve(m.B.eval()))));
                    }
                }
        };

        /*!
         * \class   DLQ
         * \author  Jonatan Olofsson    [jonatan.olofsson@gmail.com]
         * \date    2011
         * \brief   A Dynamic Linear Quadratic Regulator
         *
         * A Dynamic LQ uses a spe
         *
         * @see     LQ
         */
        template<int number_of_states, int number_of_controls>
        class DLQ : public LQ<number_of_states, number_of_controls> {
            typedef LQ<number_of_states, number_of_controls> parent; ///< Define the parent class. Used for shorter notation only
            public:
                typedef linear_model<number_of_states, number_of_controls> controller_model; ///< Define the linear model type
                typedef Matrix<double, number_of_states, 1> state_vector; ///< A state (propagation) matrix describes how the model state evolves in time, given no control
                typedef Matrix<double, number_of_controls, 1> reference_vector; ///< A reference vector contains the desired state of the controlled signals.
                typedef Matrix<double, number_of_controls, 1> control_vector; ///< A control vector contains information about the controller output
                typedef void(*lmodel_fun)(controller_model&, const state_vector&, const control_vector&); ///< Defines the function handle to call for re-linearization of model

            protected:
                lmodel_fun linearize; ///< Function-handle to the model relinearization-function

            public:
                /*!
                 * \brief   Empty default constructor
                 */
                DLQ() : parent() {}

                /*!
                 * \brief   Construct the controller with a specified linearization function
                 * \param   f   Reliearization-function
                 */
                DLQ(lmodel_fun f) : parent() {
                    set(f);
                }

                /*!
                 * \brief   Set a new relinearization-function
                 * \param   f   Reliearization-function
                 */
                void set_model(lmodel_fun f) {
                    linearize = f;
                }

                /*!
                 * \brief   Calculate the control signal optimal for following reference
                 * \param   x   Current state of system
                 * \param   r   Reference to bring the system to
                 * \return  Optimal control signal for following reference, given cost matrices of model
                 */
                const control_vector& control_signal(const state_vector& x, const reference_vector& r) {
                    linearize(parent::m, x, parent::u);
                    parent::calculate_control_matrices();
                    return parent::control_signal(x,r);
                }

                /*!
                 * \brief   Calculate the control signal optimal for bringing the system to the origin
                 * \param   x   Current state of system
                 * \return  Optimal control signal, given cost matrices of model
                 */
                const control_vector& control_signal(const state_vector& x) {
                    linearize(parent::m, x, parent::u);
                    parent::calculate_control_matrices();
                    return parent::control_signal(x);
                }
        };





        /*!
         * \class   LQ
         * \author  Jonatan Olofsson    [jonatan.olofsson@gmail.com]
         * \date    2011
         * \brief   A Linear Quadratic Regulator
         *
         * A LQR is used to calculate an optimal control signal to bring a linear
         * system to a desired state, given the system model and an optional reference.
         * The control signal is optimal in the sense that it minimizes a cost
         * function parametrized by the cost matrices of the model, where a cost
         * is defined for each combination of states and control signals respectively.
         */
        template<int number_of_states, int number_of_controls>
        class SLQ {
            public:
                typedef Matrix<double, number_of_controls, 1> control_vector; ///< A control vector contains information about the controller output
                typedef Matrix<double, number_of_states, 1> state_vector; ///< A state vector describes the state in which the system is (currently) in
                typedef Matrix<double, number_of_controls, 1> reference_vector; ///< A reference vector contains the desired state of the controlled signals.
                typedef Matrix<double, number_of_controls, number_of_states> feedback_matrix; ///< A feedback matrix is used to calculate the optimal control signal, given a current state fo the system, to bring the system to a zero-state
                typedef Matrix<double, number_of_controls, number_of_controls> feedforward_matrix; ///< A feedforward matrix is used to calculate the gain needed on the reference signal to optimally follow the reference
                typedef void(*sched_fun)(feedback_matrix&, feedforward_matrix&, const state_vector&, const reference_vector&); ///< Defines the function handle to call for scheduling of control matrices

            protected:
                control_vector u; ///< u is used for storing the computed control signal
                feedback_matrix L; ///< Feedback matrix
                feedforward_matrix Lr_inv; ///< For numerical reasons, the inverse of the feedforward matrix Lr is stored
                sched_fun schedule;

            public:

                /*!
                 * \author  Jonatan Olofsson
                 * \date    2011
                 * \brief   Default constructor
                 */
                SLQ(sched_fun f) {
                    u.setZero();
                    schedule = f;
                }

                /*!
                 * \brief   Calculate the control signal optimal for following reference
                 * \param   x   Current state of system
                 * \param   r   Reference to bring the system to
                 * \return  Optimal control signal for following reference, given cost matrices of model
                 */
                const control_vector& control_signal(const state_vector& x, const reference_vector& r) {
                    /// [2] eq. 9.11: u = Lr^-1*r - L*x
                    schedule(L, Lr_inv, x, r);
                    u = Lr_inv*r - L*x;
                    return u;
                }

                /*!
                 * \brief   Calculate the control signal optimal for bringing the system to the origin
                 * \param   x   Current state of system
                 * \return  Optimal control signal, given cost matrices of model
                 */
                const control_vector& control_signal(const state_vector& x) {
                    /// [2] eq. 9.11: u = L*x, r = 0
                    reference_vector r; r.setZero();
                    schedule(L, Lr_inv, x, r);
                    u = L*x;
                    return u;
                }

                /*!
                 * \brief   Get the current output signal
                 * \return  The last calculated control signal
                 */
                const control_vector& current_control_signal() {
                    return u;
                }
        };
    }
}
/// [2]: Glad & Ljung, Reglerteori (Studentlitteratur, 2003)

#endif
