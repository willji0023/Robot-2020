// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

namespace frc3512 {
namespace Constants {

namespace Robot {
constexpr int kDriveStick1Port = 0;
constexpr int kDriveStick2Port = 1;
constexpr int kAppendageStickPort = 2;
constexpr int kAppendageStick2Port = 3;
}  // namespace Robot

namespace Climber {

// Side Elevator Motors
constexpr int kArmPortL = 4;
constexpr int kArmPortR = 5;
// Transverser Motor
constexpr int kTransverserPort = 6;
//Transverser Set Values; Note: Set values are not yet fixed.
constexpr double kMotorIdle = 0.0;
constexpr double kMotorForward = 1.0;
constexpr double kMotorReverse = -1.0;
// Wrangler Winch Motor
constexpr int kWinchPort = 7;

}  // namespace Climber

}  // namespace Constants
}  // namespace frc3512
