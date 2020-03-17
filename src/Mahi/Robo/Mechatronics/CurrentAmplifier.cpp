#include <Mahi/Robo/Mechatronics/CurrentAmplifier.hpp>
#include <Mahi/Util/Logging/Log.hpp>

using namespace mahi::util;

namespace mahi {
namespace robo {

CurrentAmplifier::CurrentAmplifier() :
    CurrentAmplifier("UNAMED_CURRENT_AMPLIFIER", 1, nullptr, 1, nullptr, Limiter()) {}

CurrentAmplifier::CurrentAmplifier(const std::string& name, char enable_level,
                                   char* enable_channel, double command_gain,
                                   double* command_channel, Limiter current_limiter,
                                   char fault_level, const char* fault_channel,
                                   double sense_gain, const double* sense_channel) :
    Device(name),
    enable_level_(enable_level),
    fault_level_(fault_level),
    command_gain_(command_gain),
    sense_gain_(sense_gain),
    enable_channel_(enable_channel),
    fault_channel_(fault_channel),
    command_channel_(command_channel),
    sense_channel_(sense_channel),
    current_command_(0.0),
    current_limiter_(current_limiter) 
{}

void CurrentAmplifier::bind_enable_channel(char* enable_channel) {
    enable_channel_ = enable_channel;
}

void CurrentAmplifier::bind_command_channel(double* command_channel) {
    command_channel_ = command_channel;
}

void CurrentAmplifier::bind_fault_channel(const char* fault_channel) {
    fault_channel_ = fault_channel;
}

void CurrentAmplifier::bind_sense_channel(const double* sense_channel) {
    sense_channel_ = sense_channel;
}

void CurrentAmplifier::set_enable_level(char enable_level) { enable_level_ = enable_level; }

void CurrentAmplifier::set_fault_level(char fault_level) { fault_level_ = fault_level; }

void CurrentAmplifier::set_command_gain(double command_gain) { command_gain_ = command_gain; }

void CurrentAmplifier::set_sense_gain(double sense_gain) { sense_gain_ = sense_gain; }

void CurrentAmplifier::set_limiter(Limiter current_limiter) { current_limiter_ = current_limiter; }

void CurrentAmplifier::set_current(double current) {
    current_command_ = current;
    if (command_channel_)
        *command_channel_ = current_limiter_.limit(current_command_) / command_gain_;
    else
         LOG(Warning) << "CurrentAmplifier " << name() << " current command channel is invalid.";
}

double CurrentAmplifier::get_current_command() const { return current_command_; }

double CurrentAmplifier::get_current_limited() const {
    return current_limiter_.get_limited_value();
}

double CurrentAmplifier::get_current_sense() const {
    if (sense_channel_)
        return sense_gain_ * *sense_channel_;
    LOG(Warning) << "CurrentAmplifier " << name() << " current sense channel is invalid.";
    return 0.0;
}

bool CurrentAmplifier::is_faulted() {
    if (fault_channel_) {
        if (*fault_channel_ == fault_level_)
            return true;
        else
            return false;
    }
    LOG(Warning) << "CurrentAmplifier " << name() << " fault channel is invalid.";
    return true;
}

bool CurrentAmplifier::on_enable() {
    current_limiter_.reset();
    if (command_channel_)
        *command_channel_ = 0.0;
    if (enable_channel_)
        *enable_channel_ = enable_level_;
    else 
        LOG(Warning) << "CurrentAmplifier " << name() << " enable channel is invalid.";
    return true;
}

bool CurrentAmplifier::on_disable() {
    if (command_channel_)
        *command_channel_ = 0.0;
    if (enable_channel_) {
        if (enable_level_ == 1)
            *enable_channel_ = 0;
        else if (enable_level_ == 0)
            *enable_channel_ = 1;
    }
    else 
        LOG(Warning) << "CurrentAmplifier " << name() << " enable channel is invalid.";
    return true;
}

}  // namespace robo
}  // namespace mahi
