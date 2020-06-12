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
//            Craig McDonald (craig.g.mcdonald@gmail.com)

#pragma once

#include <Mahi/Robo/Mechatronics/CurrentAmplifier.hpp>

namespace mahi {
namespace robo {

/// Encapsulates a brushed or brushless DC Motor
class DcMotor : public util::Device {
public:
    /// Default Constructor
    DcMotor();
    /// Constructor
    DcMotor(const std::string& name, double kt, CurrentAmplifier* amplifier,
            Limiter current_limiter = Limiter());
    /// Sets the desired torque to be generated at the Motor, converts from
    /// torque to current, and calls set_current()
    void set_torque(double torque);
    /// Returns the attempted command current since the last call to    /// set_current()
    double get_torque_command() const;
    /// Returns the limited command current since the last call to set_current()
    double get_torque_limited() const;
    /// Returns the torque sense of the motor if it is available
    double get_torque_sense() const;
    /// Returns the amplifier on the Motor
    CurrentAmplifier* get_amplifier();

protected:
    /// Enables the Motor using the associated digital output channel
    bool on_enable() override;
    /// Disables the Motor using using the associated digital output channel and
    /// writes zero to the associated analog output channel
    bool on_disable() override;

public:
    double kt;  ///< torque constant of the Motor [torque/current]
protected:
    double torque_command_;      ///< Stores the Actuator torque since the
                                         ///< last call to set_torque()
    CurrentAmplifier* amplifier_;        ///< the current amplifier controlling the motor
    Limiter           current_limiter_;  ///< the Motor current limiter
};

}  // namespace robo
}  // namespace mahi
