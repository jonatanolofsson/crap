#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "modules/model/model.hpp"
#include "crap/module.hpp"
#include <fstream>
#include <Eigen/Core>
#include "modules/controller/model.hpp"
#include "modules/model/model.hpp"
#include "math/math.hpp"


#define eul2quat(phi,theta,psi) (sin(phi/2)*cos(theta/2)*cos(psi/2) - cos(phi/2)*sin(theta/2)*sin(psi/2)),(cos(phi/2)*sin(theta/2)*cos(psi/2) + sin(phi/2)*cos(theta/2)*sin(psi/2)),(cos(phi/2)*cos(theta/2)*sin(psi/2) - sin(phi/2)*sin(theta/2)*cos(psi/2)),(cos(phi/2)*cos(theta/2)*cos(psi/2) + sin(phi/2)*sin(theta/2)*sin(psi/2))
static const float deg = M_PI/180;

using namespace Eigen;
using namespace CRAP;
using namespace CRAP::time;
using namespace CRAP::observer;
using namespace CRAP::observer::model;

void calculate_control_signal(const model::state_vector&);
void init_controller();
void linearize(const state_vector&, const control_vector&);
void controller_tests();

namespace CRAP {
    boost::timer starting_time;
}

state_vector x;
state_vector xdot;
CRAP::controller::control_vector u;

void init_x() {
    x.setZero();
    static const state::scalar a = 150;
    x <<
    0,0,0,
    0,0,0,
    -a, a, -a, a,
    //~ 0.9961946980917455,0,-0.08715574274765817,0,
    eul2quat(5.0/180.0*M_PI, 0, 0),
    0,0,0,
    0,0,0,
    0,0,0;
    u << -a, a, -a, a;
}

void simulate() {
    frequency_t freq(30.0);
    base_float_t T = freq.dt;

    std::ofstream dxout("sim_xdot.mat");
    std::ofstream xout("sim_x.mat");

    while(ticktock(freq)) {
        xout << x << std::endl;
        xdot = fc(x,u);
        dxout << xdot << std::endl;
        x += T*xdot;
    }
}

void repeat() {
    init_x();
    std::ofstream xout("rep_x.mat");
    std::ofstream dxout("rep_xdot.mat");
    for(int i = 0; i < 5; ++i) {
        xout << x.transpose() << std::endl;
        dxout << f(x,u).transpose() << std::endl;
    }
}

void step() {
    init_x();
    state_vector x0(x);
    state_vector x(x0);
    std::cout << "::::: X0 ::::::" << std::endl << x0.transpose() << std::endl << std::endl;
    const base_float_t delta = 1e-3;
    const base_float_t div = 0.5/delta;

    std::ofstream xout("step_x.mat");
    std::ofstream dxout("step_xdot.mat");
    std::ofstream uout("step_u.mat");

    //~ for(int i = 0; i < observer::model::number_of_states; ++i) {
    {
        int i = 3;
        x[i] += delta;
        xout << x.transpose().format(IOFormat(FullPrecision)) << std::endl;
        dxout << fc(x,u).transpose().format(IOFormat(FullPrecision)) << std::endl;
        uout << u.transpose().format(IOFormat(FullPrecision)) << std::endl;
        x[i] = x0[i]-delta;
        xout << x.transpose().format(IOFormat(FullPrecision)) << std::endl;
        dxout << fc(x,u).transpose().format(IOFormat(FullPrecision)) << std::endl;
        uout << u.transpose().format(IOFormat(FullPrecision)) << std::endl;
        x[i] = x0[i];
    }
}
#define SQR(x) ((x)*(x))

void one() {
    init_x();
    std::cout << "Roll: " << std::atan2(
                2*(
                    x[state::quaternion_part_real] * x[state::quaternion_part_vector[0]]
                    + x[state::quaternion_part_vector[1]] * x[state::quaternion_part_vector[2]]
                ),
                1 - 2*(SQR(x[state::quaternion_part_vector[0]]) + SQR(x[state::quaternion_part_vector[1]]))
            )/deg << std::endl;

                std::cout << "Pitch: " << asin(2*(x[state::quaternion_part_real] * x[state::quaternion_part_vector[1]]
                - x[state::quaternion_part_vector[2]] * x[state::quaternion_part_vector[0]]))/deg << std::endl;

    std::cout << "::::: X0 ::::::" << std::endl << x.transpose() << std::endl << std::endl;
    std::cout << "::::: xdot ::::::::" << std::endl << fc(x,u).transpose() << std::endl;
}
void linear() {
    init_x();
    linearize(x,u);
}

int main(int argc_, char* argv_[])
{
    init_x();

    //~ repeat();
    //~ step();
    //~ one();
    //~ linear();

    controller_tests();
}
