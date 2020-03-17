#include <Mahi/Robo/Control/PidController.hpp>
#include <Mahi/Util/Math/Functions.hpp>

namespace mahi {
namespace robo {

PidController::PidController(double _kp, double _ki, double _kd) :
    kp(_kp),
    ki(_ki),
    kd(_kd)
{ 
    reset();
}

double PidController::operator()(double x_ref, double x, util::Time t) {
    return calculate(x_ref, x, t);
}

double PidController::calculate(double x_ref, double x, util::Time t) {
    double e = x_ref - x;
    double ei = integrator.update(e, t);
    double ed = differentiator.update(e, t);
           ed = filter.update(ed);
    return kp*e + ki*ei + kd*ed;
}

double PidController::operator()(double x_ref, double x, double xdot, util::Time t) {
    return calculate(x_ref, x, xdot, t);
}

double PidController::calculate(double x_ref, double x, double xdot, util::Time t) {
    double e = x_ref - x;
    double ei = integrator.update(e, t);
    double ed = 0 - xdot;
           ed = filter.update(ed);
    return kp * e + ki * ei + kd * ed;
}

void PidController::reset() {
    integrator.reset();
    differentiator.reset();
}


} // namespace robo
} // namespace mahi
