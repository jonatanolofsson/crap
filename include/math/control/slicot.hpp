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

#ifndef CRAP_MATH_CONTROL_SLICOT_HPP_
#define CRAP_MATH_CONTROL_SLICOT_HPP_

#include "math/control/linear_model.hpp"
#include <Eigen/Core>
#include <Eigen/LU>

namespace CRAP {
    namespace control {
        using namespace Eigen;

        extern"C" {
            /*!
             *  Riccati-equation solver
             * http://www.slicot.org/shared/doc/SB02MD.html
             *
             * DICO     char        char*   'C'ontinuous or 'D'iscrete Riccati eq.
             * HINV     char        char*   'D'irect or 'I'nverse of H - which is calculated - inverse is always faster
             * UPLO     char        char*   'U'pper or 'L'ower triangle of G and Q is stored
             * SCAL     char        char*   'G'eneral or 'N'o scaling applied
             * SORT     char        char*   'S'table or 'U'nstable eigenvalues comes first
             * N        int         int*    Number of states
             * A     dbl[LDA,N]     dbl*    State matrix
             * LDA      int         int*    Leading dimension of A ???
             * G     dbl[LDG,N]     dbl*    Upper or lower triangle of BR\B'
             * LDG      int         int*    Leading dimension of G
             * Q     dbl[LDQ,N]     dbl*    [NxN] first should contain upper/lower triangle of Q
             * LDQ      int         int*    Leading dimenstion of Q
             * RCOND    dbl         dbl*    [output] Estimated condition
             * WR       dbl[2N]     dbl*    [output] Eigenvalues of S, real part
             * WI       dbl[2N]     dbl*    [output] Eigenvalues of S, imaginary part
             * S    dbl[LDS,2N]     dbl*    Schur form of H
             * LDS      int         int*    Leading dimension of S
             * U    dbl[LDU,2N]     dbl*    Schur transformation matrix
             * LDU      int         int*    Leading dimension of U
             * IWORK    int[2N]     int*
             * DWORK dbl[LDWORK]    dbl*
             * LDWORK   int         int*    >= MAX(3,6*N) for discrete case. Preferably larger.
             * BWORK   logic[2N]    int*
             * INFO     int         int*
             */
            void sb02md_(
                char*,char*,char*,char*,char*,
                int*,double*,int*,double*,int*,double*,int*,double*,
                double*,double*,double*,
                int*,double*,
                int*,int*,double*,
                int*,int*,int*
            );

            /*!
             *  Riccati-equation solver
             * http://www.slicot.org/shared/doc/SB02OD.html
             *
             * DICO     char        char*   'C'ontinuous or 'D'iscrete Riccati eq.
             * JOBB     char        char*   'B' and R matrices given, or 'G'
             * FACT     char        char*   Factorization of Q and R
             * UPLO     char        char*   'U'pper or 'L'ower triangle of G and Q is stored
             * JOBL     char        char*   L is 'Z'ero or 'N'on-zero
             * SORT     char        char*   'S'table or 'U'nstable eigenvalues comes first
             * N        int         int*    Number of states
             * M        int         int*    Number of inputs
             * P        int         int*    Number of outputs
             * A     dbl[LDA,N]     dbl*    State matrix
             * LDA      int         int*    Leading dimension of A
             * G     dbl[LDG,N]     dbl*    Upper or lower triangle of BR\B'
             * LDG      int         int*    Leading dimension of G
             * Q     dbl[LDQ,N]     dbl*    [NxN] first should contain upper/lower triangle of Q
             * LDQ      int         int*    Leading dimenstion of Q
             * R     dbl[LDR,M]     dbl*    Not used
             * LDR      int         int*    Leading dimenstion of R
             * L     dbl[LDL,M]     dbl*    L-matrix
             * LDL      int         int*    Leading dimenstion of L
             * RCOND    dbl         dbl*    [output] Estimated condition
             * X        dbl[LDX,N]  dbl*    [output] X
             * LDX      int         int*    Leading dimenstion of X
             * WR       dbl[2N]     dbl*    [output] not used
             * WI       dbl[2N]     dbl*    [output] not used
             * BETA     dbl[2N]     dbl*    [output] not used
             * S    dbl[LDS,2N]     dbl*    Schur form of H
             * LDS      int         int*    Leading dimension of S
             * T    dbl[LDT,2N]     dbl*    Reduced matrix pencil
             * LDT      int         int*    Leading dimension of T
             * U    dbl[LDU,2N]     dbl*    Schur transformation matrix
             * LDU      int         int*    Leading dimension of U
             * TOL      dbl         dbl*    Not used.
             * IWORK    int[2N]     int*    >= MAX(1,2*N)   if JOBB = 'G'
             * DWORK dbl[LDWORK]    dbl*
             * LDWORK   int         int*    >= MAX(7*(2*N+1)+16,16*N)
             * BWORK   logic[2N]    int*
             * INFO     int         int*
             */
            void sb02od_(
                char*,char*,char*,char*,char*,char*,//SORT
                int*,int*,int*,//P
                double*,int*,double*,int*,double*,int*,double*,int*,double*,int*,//Ldim
                double*,double*,int*,//Xdim
                double*,double*,double*,//BETA
                double*,int*,double*,int*,double*,int*,//Udim
                double*,int*,double*,//DWORK
                int*,int*,int*//INFO
            );


            /*!
             * Calculate the matrix exponential
             * http://www.slicot.org/shared/doc/MB05OD.html
             *
             * BALANC   char        char*   Perform 'S'caling or do 'N'ot
             * N        int         int*    Order of matrix A
             * NDIAG    int         int*    Diagonal Pade approximant order. Should be 9 if unknown
             * delta    dbl         dbl*    Scalar delta value of problem
             * A        dbl[LDA,N]  dbl*    [input/output] Send in the A-matrix and get the result back
             * LDA      int         int*    Leading dimension of A
             * MDIG     int         int*    Accurate digits of 1-norm of solution
             * IDIG     int         int*    Accurate digits of 1-norm of solution to 95 % confidence
             * IWORK    int[N]      int*
             * DWORK dbl[LDWORK]    dbl*
             * LDWORK   int         int*    >= N*(2*N+NDIAG+1)+NDIAG
             * IWARN    int         int*
             * INFO     int         int*
             */
            void mb05od_(
                char*, int*, int*, double*, double*, int*, int*, int*, int*, double*, int*, int*, int*
            );
        }


        /*!
        * \brief    Solve the Algebraic Riccati Equation (CARE/DARE)
        * \param    m   Linear model of the system
        * \param    Q   Matrix with the weights on the system states
        * \param    R   Matrix with the weights on the control signals
        * \param    X   This matrix will contain the result of the DARE equation upon finish
        * \return Error indicator:
        *     INFO    INTEGER
        *             = 0:  successful exit;
        *             < 0:  if INFO = -i, the i-th argument had an illegal
        *                   value;
        *             = 1:  if the computed extended matrix pencil is singular,
        *                   possibly due to rounding errors;
        *             = 2:  if the QZ (or QR) algorithm failed;
        *             = 3:  if reordering of the (generalized) eigenvalues
        *                   failed;
        *             = 4:  if after reordering, roundoff changed values of
        *                   some complex eigenvalues so that leading eigenvalues
        *                   in the (generalized) Schur form no longer satisfy
        *                   the stability condition; this could also be caused
        *                   due to scaling;
        *             = 5:  if the computed dimension of the solution does not
        *                   equal N;
        *             = 6:  if a singular matrix was encountered during the
        *                   computation of the solution matrix X.
        */
        template<int number_of_states, int number_of_controls>
        int are(const linear_model<number_of_states, number_of_controls>& m, Matrix<double, number_of_states, number_of_states>& X) {
            typedef Matrix<double, number_of_states, number_of_states> state_matrix;
            typedef Matrix<double, number_of_controls, number_of_controls> state_weight_matrix;
            X = m.M.transpose()*m.Q*m.M;
            state_matrix A = m.A;

            // Protect model in memory from the Fortran
            char DICO = m.DICO;

            // Inverse is faster, and we don't use H
            char HINV = 'I';

            char UPLO = 'U';

            char SCAL = 'G';

            //~ To obtain a stabilizing solution of the algebraic Riccati
            //~ equation for DICO = 'D', set SORT = 'U', if HINV = 'D', or set
            //~ SORT = 'S', if HINV = 'I'.
            char SORT = 'S';

            int N = number_of_states;
            int LDA = number_of_states;
            state_matrix G = (m.B*m.R.partialPivLu().solve(m.B.transpose().eval()));
            int LDG = number_of_states;
            int LDQ = number_of_states;

            double RCOND;
            double WR[2*number_of_states];
            double WI[2*number_of_states];
            double S[2*number_of_states][2*number_of_states];
            int LDS = 2*number_of_states;
            double U[2*number_of_states][2*number_of_states];
            int LDU = 2*number_of_states;

            int IWORK[2*number_of_states];
            int BWORK[2*number_of_states];
            int INFO = 0;
            {
                int LDWORK = 12*number_of_states;
                double DWORK[12*number_of_states];
                sb02md_(
                    &DICO, &HINV, &UPLO, &SCAL, &SORT,
                    &N, A.data(), &LDA, G.data(), &LDG,
                    X.data(), &LDQ, &RCOND, WR, WI,
                    (double*)S, &LDS, (double*)U, &LDU,
                    IWORK, DWORK, &LDWORK, BWORK, &INFO
                );
            }
            //~ std::cout << "InfoMD: " << INFO << std::endl;

            if(INFO == 1) {
                int LDWORK = 8*(2*number_of_states+1)+20;
                double DWORK[8*(2*number_of_states+1)+20];
                char JOBB = 'G';
                char JOBL = 'Z';
                char FACT = 'N';
                int M = number_of_controls;
                int P = 0;//Not used if FACT = N
                double L[1];
                int LDL = number_of_controls;
                double R[1];
                int LDR = number_of_controls;
                double BETA[2*number_of_states];
                double T[2*number_of_states][2*number_of_states];
                int LDT = 2*number_of_states;
                int LDX = number_of_states;
                double TOL = 1e-5;

                state_weight_matrix Qint = m.Q;

                sb02od_(
                    &DICO, &JOBB, &FACT, &UPLO, &JOBL, &SORT,//6
                    &N, &M, &P,//9
                    A.data(), &LDA, G.data(), &LDG, Qint.data(), &LDQ, R, &LDR, L, &LDL,//19
                    &RCOND, X.data(), &LDX,//22
                    WR, WI, BETA,//25
                    (double*)S, &LDS, (double*)T, &LDT, (double*)U, &LDU,//31
                    &TOL,IWORK,DWORK,&LDWORK,BWORK,&INFO
                );
                //~ std::cout << "InfoMO: " << INFO << std::endl;
            }

            return INFO;
        }


        /*!
        * \brief    Calculate the matrix exponential
        * \param      A     [input/output] Insert the state matrix A here and retrieve the matrix exponential of it here
        * \param    delta   Calculate exp(A*delta), delta beeing a scalar
        * \return   Error indicator:
        *     INFO    INTEGER
        *             = 0:  successful exit;
        *             < 0:  if INFO = -i, the i-th argument had an illegal
        *                   value;
        *             = 1:  if the norm of matrix A*delta (after a possible
        *                   balancing) is too large to obtain an accurate
        *                   result;
        *             = 2:  if the coefficient matrix (the denominator of the
        *                   Pade approximant) is exactly singular; try a
        *                   different value of NDIAG;
        *             = 3:  if the solution exponential would overflow, possibly
        *                   due to a too large value DELTA; the calculations
        *                   stopped prematurely. This error is not likely to
        *                   appear.
        */
        template<int num>
        int mexp(Matrix<double, num, num>& A, double delta) {
            const int ndiag = 9;
            char BALANC = 'N';
            int N = num;
            int NDIAG = ndiag;
            int LDA = num;
            int MDIG, IDIG,IWARN,INFO;
            int IWORK[num];
            int LDWORK = num*(2*num+ndiag+1)+ndiag;
            double DWORK[num*(2*num+ndiag+1)+ndiag];
            mb05od_(&BALANC, &N, &NDIAG, &delta, A.data(), &LDA, &MDIG, &IDIG, IWORK, DWORK, &LDWORK, &IWARN, &INFO);
            return INFO;
        }

        /*!
         * \brief   Convert a continuous state-space system to a discrete equivalent
         * \param   A   Continous state derivative description
         * \param   B   Continous control derivative description
         * \param   Ad  [Output] Discrete state description
         * \param   Bd  [Output] Discrete control description
         * \param   Ts  Sample-time with which to discretize the system
         */
        template<int number_of_states, int number_of_controls>
        void c2d(
                const Matrix<double, number_of_states, number_of_states>& A,
                const Matrix<double, number_of_states, number_of_controls>& B,
                Matrix<double, number_of_states, number_of_states>& Ad,
                Matrix<double, number_of_states, number_of_controls>& Bd,
                const double Ts) {

            Matrix<double, number_of_states + number_of_controls, number_of_states + number_of_controls> eAT;
            eAT.template block<number_of_states, number_of_states>(0,0) = A;
            eAT.template block<number_of_states, number_of_controls>(0,number_of_states) = B;
            mexp(eAT, Ts);
            Ad = eAT.template block<number_of_states, number_of_states>(0,0);
            Bd = eAT.template block<number_of_states, number_of_controls>(0,number_of_states);
        }
    }
}

#endif
