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
// Author(s): Craig McDonald (craig.g.mcdonald@gmail.com)
//            Evan Pezent (epezent@rice.edu)

#pragma once

#include <iostream>
#include <Mahi/Robo/Types.hpp>
#include <Mahi/Util/Logging/Log.hpp>
#include <vector>

namespace mahi {
namespace robo {

class ForceSensor {
public:
    /// Default constructor
    ForceSensor();
    /// Destructor
    virtual ~ForceSensor();
    /// Returns force along speficied axis
    virtual double get_force(Axis axis) = 0;
    /// Returns forces along X, Z, and Z axes
    virtual std::vector<double> get_forces() = 0;
    void zero(){
        LOG(mahi::util::Warning) << "Using the default template zero in ForceSensor.hpp in mahi-robo. It is an empty function.";
    }

protected:
    std::vector<double> forces_;  ///< measured forces
};

}  // namespace robo
}  // namespace mahi
