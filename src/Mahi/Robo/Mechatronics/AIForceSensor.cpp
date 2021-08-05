#include <Mahi/Robo/Mechatronics/AIForceSensor.hpp>
#include <Mahi/Util/Logging/Log.hpp>
#include <Mahi/Util/System.hpp>
#include <fstream>

using namespace mahi::util;

namespace mahi {
namespace robo {

AIForceSensor::AIForceSensor() :
    bias_(0),
    zeroedValue_(0)
{}
// quadratic fit for voltage-force calibration (a + bx + cx^2)
AIForceSensor::AIForceSensor(const double* ch, const double calA, const double calB, const double calC) :
    channel_(ch) 
{
    set_force_calibration(calA,calB,calC);
}

void AIForceSensor::set_channel(const double* ch) {
    channel_ = ch;
}

void AIForceSensor::set_force_calibration(double a, double b, double c) {
    calibration_ = {a,b,c};
}

double AIForceSensor::get_force(Axis axis) {
    update_biased_voltages();
    switch (axis) {
        case AxisX: 
            return calibration_[0] + zeroedValue_ * calibration_[1] + zeroedValue_ * zeroedValue_ * calibration_[2];
        default: 
            LOG(Warning) << "AIForceSensor class only setup to work with AxisX. Returning 0";
            return 0.0;
    }
}

std::vector<double> AIForceSensor::get_forces() {
    update_biased_voltages();
    forces_[0] = calibration_[0] + zeroedValue_ * calibration_[1] + zeroedValue_ * zeroedValue_ * calibration_[2];
    forces_[1] = 0.0;
    forces_[2] = 0.0;
    LOG(Warning) << "AIForceSensor class only setup to work with AxisX. Returning {force, 0, 0}";
    return forces_;
}

void AIForceSensor::zero() {
    bias_ = *channel_;
}

void AIForceSensor::update_biased_voltages() {
    zeroedValue_ = *channel_ - bias_;
}

}  // namespace robo
}  // namespace mahi
