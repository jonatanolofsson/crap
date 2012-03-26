
#include "modules/controller/model.hpp"
#include "modules/model/model.hpp"
#include "yaml-cpp/yaml.h"
#include "math/control/LQ.hpp"
#include <Eigen/Core>
#include <fstream>

using namespace Eigen;
using namespace CRAP;
using namespace CRAP::observer;
using namespace CRAP::observer::model;
controller::reference_vector r;
extern controller::control_vector u;
extern state_vector x;
extern void init_x();

YAML::Node config;

std::ofstream Aout("A.mat");
std::ofstream Bout("B.mat");
std::ofstream Qout("Q.mat");
std::ofstream Rout("R.mat");
std::ofstream rout("r.mat");
std::ofstream Lout("L.mat");
std::ofstream Lrout("Lr.mat");
std::ofstream eXout("eX.mat");
std::ofstream duout("du.mat");
//~ #ifdef CRAP_PLOT
    //~ using namespace cpplot;
    //~ static const int count = 30*10;
    //~ auto u0plot = figure("Control signal")->subplot(2,2,1)->title("u0")->add<Line>()->set_capacity(count);
    //~ auto u1plot = figure("Control signal")->subplot(2,2,2)->title("u1")->add<Line>()->set_capacity(count);
    //~ auto u2plot = figure("Control signal")->subplot(2,2,3)->title("u2")->add<Line>()->set_capacity(count);
    //~ auto u3plot = figure("Control signal")->subplot(2,2,4)->title("u3")->add<Line>()->set_capacity(count);
//~ #endif



        typedef CRAP::control::LQ<model::number_of_states+1, controller::model::number_of_controls, controller::model::number_of_references> controller_type;
        typedef controller_type::controller_model lmodel_t;
        lmodel_t lmodel;
        controller_type::state_vector extended_x;
        controller_type regulator;

        void linearize(const state_vector& x0, const control_vector& u) {
            Block<lmodel_t::state_matrix, controller::model::number_of_linearized_states, controller::model::number_of_linearized_states> A(lmodel.A, 0, 0);
            Block<lmodel_t::control_matrix, controller::model::number_of_linearized_states, controller::model::number_of_controls> B(lmodel.B, 0, 0);
            ::CRAP::control::jacobians<controller::model::number_of_linearized_states, controller::model::number_of_controls, 0>(
                observer::model::fc, x0, u,
                A,
                B
            );

            lmodel.A.block<controller::model::number_of_linearized_states, 1>(0, controller::model::number_of_states)
                = observer::model::fc(x0, u).segment<controller::model::number_of_linearized_states>(0)
                    - A*x0.segment<controller::model::number_of_linearized_states>(0);
        }

        //~ std::ofstream Xlin("Xlin.mat");
        void calculate_control_signal(const state_vector& x0) {
            //~ Xlin << x0.transpose() << std::endl;
            using namespace observer::model;
            linearize(x0, u);

            extended_x.segment<controller::model::number_of_linearized_states>(0)
                = x0.segment<controller::model::number_of_linearized_states>(0);

            std::cout << "Using reference: " << r.transpose() << ". Original state: " << extended_x.segment<3>(state::velocities).transpose() << " " << extended_x(state::omega_part[Z]);

            extended_x.segment<3>(state::velocities) -= r.segment<3>(0);
            extended_x(state::omega_part[Z]) -= r(3);
            std::cout << " New state: " << extended_x.segment<3>(state::velocities).transpose() << " " << extended_x(state::omega_part[Z]) << std::endl;

            regulator.schedule(lmodel);
//~
    Aout << lmodel.A.format(IOFormat(FullPrecision)) << std::endl;
    Bout << lmodel.B.format(IOFormat(FullPrecision)) << std::endl;
    Qout << lmodel.Q.format(IOFormat(FullPrecision)) << std::endl;
    Rout << lmodel.R.format(IOFormat(FullPrecision)) << std::endl;
    rout << r.transpose().format(IOFormat(FullPrecision)) << std::endl;
    Lout << regulator.L.format(IOFormat(FullPrecision)) << std::endl;
    Lrout << regulator.Lr_inv.solve(Matrix<model_float_t, 4, 4>::Identity()).format(IOFormat(FullPrecision)) << std::endl;
    eXout << extended_x.transpose().format(IOFormat(FullPrecision)) << std::endl;
    duout << regulator(extended_x).transpose().format(IOFormat(FullPrecision)) << std::endl;


    std::cout << "deltaU: " << regulator(extended_x).transpose() << std::endl;

            u += regulator(extended_x);
            //~ u += regulator(extended_x,r);

            #ifdef CRAP_PLOT
                u0plot << std::make_pair(CRAP::starting_time.elapsed(), u(0));
                u1plot << std::make_pair(CRAP::starting_time.elapsed(), u(1));
                u2plot << std::make_pair(CRAP::starting_time.elapsed(), u(2));
                u3plot << std::make_pair(CRAP::starting_time.elapsed(), u(3));
            #endif
        }

        void init_controller() {
            using namespace observer::model;

            r.setZero();

            lmodel.A(controller::model::state::roll, state::omega_part[X]) = 1;
            lmodel.A(controller::model::state::pitch, state::omega_part[Y]) = 1;
            lmodel.Q(controller::model::number_of_states,controller::model::number_of_states) = 0;
            lmodel.Q.block<4,4>(state::rotor_velocities,state::rotor_velocities).setZero();
            lmodel.Q *= 100;

            lmodel.A(controller::model::number_of_states, controller::model::number_of_states) = -1e-8;

            extended_x[controller::model::number_of_states] = 1;
            lmodel.M.block<3, 3>(0,state::velocities).setIdentity();
            lmodel.M(3, state::omega_part[Z]) = 1;
            std::cout << "::::: M ::::::" << std::endl << lmodel.M << std::endl << "::::::::::::::::::" << std::endl;
            static const state::scalar a = config["initial_rotorspeed"].as<state::scalar>(400.0);
            u << -a, a, -a, a;
        }

void test_calculate_control_signal() {
    init_x();
    calculate_control_signal(x);
}
void controller_tests() {
    init_controller();
    test_calculate_control_signal();
}
