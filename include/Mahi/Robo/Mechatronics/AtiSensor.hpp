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
#include <Mahi/Robo/Mechatronics/ForceSensor.hpp>
#include <Mahi/Robo/Mechatronics/TorqueSensor.hpp>
#include <array>
#include <string>

namespace mahi {
namespace robo {

/// Implements an ATI force/torque transducer
class AtiSensor : public ForceSensor, public TorqueSensor {
public:
    /// Calibration matrix data obtained from "UserAxis" elements
    /// in ATI supplied calibration file (e.g. "FTXXXXX.cal")
    struct Calibration {
        std::array<double, 6> Fx;
        std::array<double, 6> Fy;
        std::array<double, 6> Fz;
        std::array<double, 6> Tx;
        std::array<double, 6> Ty;
        std::array<double, 6> Tz;
    };

public:
    /// Constucts AtiSensor with unspecified channels and no calibration
    AtiSensor();
    /// Constructs AtiSensor with specified channels and loads calibration from filepath
    AtiSensor(const double* ch0, const double* ch1, const double* ch2, const double* ch3,
              const double* ch4, const double* ch5, const std::string& filepath);
    /// Constructs AtiSensor from specified channels and manual calibration
    AtiSensor(const double* ch0, const double* ch1, const double* ch2, const double* ch3,
              const double* ch4, const double* ch5, Calibration calibration);
    /// Sets the voltages channels associated with this ATI sensor
    void set_channels(const double* ch0, const double* ch1, const double* ch2, const double* ch3,
                      const double* ch4, const double* ch5);
    /// Loads calibration from ATI calibration file (e.g. "FTXXXXX.cal")
    bool load_calibration(const std::string& filepath);
    /// Allows for manually setting calibration
    void set_calibration(Calibration calibration);
    /// Returns force along specified axis
    double get_force(Axis axis) override;
    /// Returns forces along X, Z, and Z axes
    std::vector<double> get_forces() override;
    /// Returns torque along specifed axis
    double get_torque(Axis axis) override;
    /// Returns torque along X, Z, and Z axes
    std::vector<double> get_torques() override;
    /// Zeros all forces and torques at current preload
    void zero() override;

private:
    /// Updates biased voltages
    void update_biased_voltages();

private:
    std::vector<const double*> channels_;     ///< raw voltage channels
    Calibration                calibration_;  ///< calibration matrix
    std::array<double, 6>      bias_;         ///< bias vector
    std::array<double, 6>      bSTG_;         ///< biased strain gauge voltages
};

}  // namespace robo
}  // namespace mahi
