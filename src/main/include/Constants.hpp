// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <units/units.h>
#include <wpi/math>

namespace frc3512::Constants {

namespace Robot {
constexpr int kDriveStick1Port = 0;
}  // namespace Robot

namespace Flywheel {
constexpr int kLeftPort = 14;
constexpr int kRightPort = 15;
constexpr int kEncoderA = 2;
constexpr int kEncoderB = 3;

constexpr double kV = 0.00957;
constexpr double kA = 0.02206;
constexpr double kMaxAngularVelocity = 1000.5;
constexpr double kAngularVelocityTolerance = 7.0;  // radians per second

constexpr double kGearRatio = 2.0;
constexpr double kDpP = (wpi::math::pi * 2.0) * kGearRatio / 20.0;
}  // namespace Flywheel

constexpr auto kDt = 0.00505_s;
constexpr int kControllerPrio = 50;
}  // namespace frc3512::Constants
