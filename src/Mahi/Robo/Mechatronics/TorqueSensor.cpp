#include <Mahi/Robo/Mechatronics/TorqueSensor.hpp>

namespace mahi {
namespace robo {

TorqueSensor::TorqueSensor() :
    torques_(3, 0.0)
{   
}

TorqueSensor::~TorqueSensor() {

}


} // namespace robo
} // namespace mahi
