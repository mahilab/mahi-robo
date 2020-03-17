// MIT License
//
// Copyright (c) 2020 Mechatronics and Haptic Interfaces Lab - Rice University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// Author(s): Evan Pezent (epezent@rice.edu)

#pragma once

#include <Mahi/Robo/Control/Limiter.hpp>
#include <Mahi/Util/Device.hpp>

namespace mahi {
namespace robo {

/// Encapsulates a DC current amplifier
class CurrentAmplifier : public util::Device {
public:
    CurrentAmplifier();
    /// Constructor
    /// #name the name of the amplifier
    /// #enable_level TTL level required to enable device
    /// #enable_channel the DO channel which enables the Amplifier
    /// #command_gain the command gain in [A/V]
    /// #command_channel the AO channel which commands current
    /// #current_limiter current Limiter protecting Amplifier
    /// #fault_level TTL level expected when the Amplifier faults
    /// #fault_channel the DI channel which reads in a fault
    /// #sense_gain the sense gain in [A/V]
    /// #sense_channel the AI channel which reads in sensed current
    CurrentAmplifier(const std::string& name, char enable_level, char* enable_channel,
                     double command_gain, double* command_channel,
                     Limiter current_limiter = Limiter(), char fault_level = 1,
                     const char* fault_channel = nullptr, double sense_gain = 1,
                     const double* sense_channel = nullptr);
    /// Sets the digital output channel which enables the Amplifer.
    void bind_enable_channel(char* enable_channel);
    /// Sets the analog output channel which commands current.
    void bind_command_channel(double* command_channel);
    /// Sets the digital input channel which reads in a fault.
    void bind_fault_channel(const char* fault_channel);
    /// Sets the analog input channel which reads in sensed current.
    void bind_sense_channel(const double* sense_channel);
    /// Sets the TTL level required to enable device.
    void set_enable_level(char enable_level);
    /// Sets the TTL level expected when the Amplifier faults.
    void set_fault_level(char fault_level);
    /// Sets the analog command gain in [A/V]
    void set_command_gain(double command_gain);
    /// Sets the analog sense gain in [A/V]
    void set_sense_gain(double sense_gain);
    /// Sets the Limiter protecting over current
    void set_limiter(Limiter current_limiter);
    /// Sets the desired current [A] to be produced by the Amplifier
    void set_current(double current);
    /// Returns the last current value commanded
    double get_current_command() const;
    /// Returns the limited version of the last current value commanded
    double get_current_limited() const;
    /// Gets the actual current measured by the amplifier
    double get_current_sense() const;
    /// Returns true if the Amplifier is in a faulted state
    bool is_faulted();

protected:
    /// Enables the Amplifier using the DO enable channel
    bool on_enable() final;
    /// Disables the Amplifier using the DO enable channel
    bool on_disable() final;

protected:
    char          enable_level_;     ///< TTL enable level
    char          fault_level_;      ///< TTL fault level
    double        command_gain_;     ///< command gain [A/V]
    double        sense_gain_;       ///< sense gain [A/V]
    char*         enable_channel_;   ///< DO enable channel
    const char*   fault_channel_;    ///< DI fault channel
    double*       command_channel_;  ///< AO current command channel
    const double* sense_channel_;    ///< AI current sense channel
    double        current_command_;  ///< current command value
    Limiter       current_limiter_;  ///< current limiter protecting Amplifier
};

}  // namespace robo
}  // namespace mahi
