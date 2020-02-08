// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <units/units.h>

namespace frc3512::Constants {
/**
 * Controller Base
 * NOTE: Default values are used here.
 */
constexpr int kControllerPrio = 50;
constexpr auto kDt = 0.00505_s;

namespace Intake {

/**
 *  Motor Set Values
 */ 
constexpr double kMotorIdle = 0.0;
constexpr double kMotorForward = 1.0;
constexpr double kMotorReverse = -1.0;

/**
 *  Arm Motor Port
 */
constexpr int kArmMotorPort = 3;

/**
 *  Conveyor Motor Port
 */
constexpr int kConveyorPort = 2;

/**
 *  Funnel Motor Port
 */
constexpr int kFunnelPort = 4;

/**
 *  Proximity Sensor Ports
 */
constexpr int kLowerAnalogPort = 0;
constexpr int kUpperAnalogPort = 1;

/**
 *  Solenoid Ports
 */
constexpr int kArmPort = 0;
}  // namespace Intake

namespace Turret {

/*
    Note: The values here are default ones for now.

    We can change these later in the future.
*/

// Spark Max Port Values
constexpr double kTurretSparkMaxPort = 0;

// Hall Sensor Ports
constexpr double kRightHallSensorPort = 0;
constexpr double kLeftHallSensorPort = 1;

// Encoder Values
constexpr int kTurretEncoderA = 1;
constexpr int kTurretEncoderB = 0;

}  // namespace Turret

}  // namespace frc3512::Constants
