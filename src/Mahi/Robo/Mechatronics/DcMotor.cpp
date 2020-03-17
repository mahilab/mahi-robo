#include <Mahi/Robo/Mechatronics/DcMotor.hpp>
#include <Mahi/Util/Math/Functions.hpp>
#include <Mahi/Util/Timing/Clock.hpp>

using namespace mahi::util;

namespace mahi {
namespace robo {

DcMotor::DcMotor() : Device("UNAMED_DC_MOTOR"), kt(0.0), amplifier_(nullptr), current_limiter_() {}

DcMotor::DcMotor(const std::string& name, double _kt, CurrentAmplifier* amplifier,
                 Limiter current_limiter) :
    Device(name), kt(_kt), amplifier_(amplifier), current_limiter_(current_limiter) {}

bool DcMotor::on_enable() {
    if (amplifier_) {
        if (amplifier_->enable())
            return true;
        else
            return false;
        }
    return false;
}

bool DcMotor::on_disable() {
    if (amplifier_) {
        if (amplifier_->disable())
            return true;
        else
            return false;
    }
    return false;
}

void DcMotor::set_torque(double torque) {
    torque_command_ = torque;
    if (amplifier_)
        amplifier_->set_current(current_limiter_.limit(torque_command_ / kt));
}

double DcMotor::get_torque_command() const { 
    return torque_command_; 
}

double DcMotor::get_torque_limited() const { 
    if (amplifier_)
        return kt * amplifier_->get_current_limited(); 
    return 0;
}

double DcMotor::get_torque_sense() const { 
    if (amplifier_)
        return kt * amplifier_->get_current_sense();
    return 0;
}

CurrentAmplifier* DcMotor::get_amplifier() { 
    return amplifier_; 
}

}  // namespace robo
}  // namespace mahi
