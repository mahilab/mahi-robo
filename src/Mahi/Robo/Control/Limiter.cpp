#include <Mahi/Robo/Control/Limiter.hpp>
#include <Mahi/Util/Math/Functions.hpp>
#include <Mahi/Util/Math/Constants.hpp>

namespace mahi {
namespace robo {

Limiter::Limiter() :
    mode_(None)
{
}

Limiter::Limiter(double abs_limit) :
    mode_(Saturate),
    min_limit_(-abs(abs_limit)),
    max_limit_(abs(abs_limit)),
    exceeded_(false)
{
}

Limiter::Limiter(double min_limit, double max_limit) :
    mode_(Saturate),
    min_limit_(min_limit),
    max_limit_(max_limit),
    exceeded_(false)
{
}

Limiter::Limiter(double continuous_limit, double abs_limit, util::Time time_limit) :
    mode_(Accumulate),
    min_limit_(-abs(abs_limit)),
    max_limit_(abs(abs_limit)),
    continuous_limit_(continuous_limit),
    setpoint_( ((abs_limit*abs_limit) - (continuous_limit*continuous_limit)) * time_limit.as_seconds() ),
    accumulator_(0.0),
    limited_value_(0.0),
    clock_(util::Clock()),
    exceeded_(false)
{ }

double Limiter::limit(double unlimited_value) {
    switch(mode_) {
        case None:
            limited_value_ = unlimited_value;
            break;
        case Saturate:
            limited_value_ = util::clamp(unlimited_value, min_limit_, max_limit_);
            break;
        case Accumulate:
            accumulator_ += ((limited_value_*limited_value_) - (continuous_limit_*continuous_limit_)) * clock_.get_elapsed_time().as_seconds();
            accumulator_ = util::clamp(accumulator_, 0.0, util::INF);
            clock_.restart();
            if (accumulator_ > setpoint_)
                limited_value_ = util::clamp(unlimited_value, continuous_limit_);
            else
                limited_value_ = util::clamp(unlimited_value, min_limit_, max_limit_);
    }
    if (limited_value_ != unlimited_value)
        exceeded_ = true;
    else
        exceeded_ = false;
    return limited_value_;
}

bool Limiter::limit_exceeded() const {
    return exceeded_;
}

double Limiter::get_limited_value() const {
    return limited_value_;
}

double Limiter::get_setpoint() const {
    return setpoint_;
}

double Limiter::get_accumulator() const {
    return accumulator_;
}

void Limiter::reset() {
    if (mode_ == Accumulate) {
        accumulator_ = 0.0;
        clock_.restart();
    }
}

} // namespace robo
} // namespace mahi
