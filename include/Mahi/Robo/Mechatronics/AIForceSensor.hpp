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
// Author(s): Janelle Clark (jc105@rice.edu)
//            Nathan Dunkelberger (nbd2@rice.edu)

#pragma once
#include <Mahi/Robo/Mechatronics/ForceSensor.hpp>
#include <array>
#include <string>

namespace mahi {
namespace robo {

class AIForceSensor : public ForceSensor {
public:
    /// Default constructor
    AIForceSensor();
    /// Constructs AIForceSensor with specified channel and loads calibration coefficients for quadratic fit
    AIForceSensor(const double* ch, const double calA, const double calB, const double calC);
    /// Destructor
    // ~AIForceSensor();
    /// Sets the voltages channels associated with this ATI sensor
    void set_channel(const double* ch);
    /// Allows for manually setting calibration, quadratic fit for voltage-force calibration (a + bx + cx^2)
    void set_force_calibration(double a, double b, double c);
    /// Returns force along specified axis
    double get_force(Axis axis = AxisX) override;
    /// Returns forces along X, Z, and Z axes
    std::vector<double> get_forces() override;
    /// Zeros all forces and torques at current preload
    void zero();

    private:
    /// Updates biased voltages
    void update_biased_voltages();

private:
    const double* channel_;            ///< raw voltage channel
    std::vector<double> calibration_;  ///< calibration constants voltage to force
    double      bias_;                 ///< bias vector
    double      zeroedValue_;          ///< biased strain gauge voltages
};

}  // namespace robo
}  // namespace mahi
