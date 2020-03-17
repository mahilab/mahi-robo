#include <MEL/Mechatronics/PositionSensor.hpp>
#include <iostream>
#include <string>

namespace mahi {
namespace robo {

//==============================================================================
// CLASS DEFINITIONS
//==============================================================================

PositionSensor::PositionSensor() :
    position_(0.0)
{ }

PositionSensor::~PositionSensor() {

}

double PositionSensor::get_position() {
    return position_;
}

} // namespace robo
} // namespace mahi
