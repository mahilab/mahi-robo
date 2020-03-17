#include <Mahi/Robo/Mechatronics/ForceSensor.hpp>

namespace mahi {
namespace robo {

ForceSensor::ForceSensor() :
    forces_(3, 0.0)
{ }

ForceSensor::~ForceSensor() {

}

} // namespace robo
} // namespace mahi
