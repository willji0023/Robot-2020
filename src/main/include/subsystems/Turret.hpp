// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <limits>

#include <frc/DigitalInput.h>
#include <frc/Encoder.h>
#include <frc/Notifier.h>
#include <rev/CANSparkMax.h>

#include "Constants.hpp"
#include "communications/PublishNode.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

/**
 *  State class which stores numerical values as "states" for the Turret to use.
 */
enum State { kIDLE = 0, kLeft, KRight, kDisabled };

/**
 * Subsystem specifically designed for the Turret (bottom, movable part of the
 * Shooter)
 */
class Turret : public SubsystemBase, public PublishNode {
public:
    Turret();
    Turret(const Turret&) = delete;
    Turret& operator=(const Turret&) = delete;

    /**
     *  Set the velocity of the Spark Max, which is wired to the Turret.
     *
     *  Can be seen as a way of having it turn in a certain direction.
     *  This depends on the value given to it.
     *
     *  @param velocity between [-1.0 ... 1.0]
     */
    void setVelocity(double velocity) {}

    /**
     *  Enables the controller.
     */
    void EnableController() {}

    /**
     *  Disables the controller
     */
    void DisableController() {}

    /**
     *  Sets a hard limit for two limit switches by using DigitalInput.
     *
     *  @param left switch
     *  @param right switch
     */
    void setHardLimit(frc::DigitalInput* forwardLimitSwitch,
                      frc::DigitalInput* reverseLimitSwitch);

    /**
     *  Sets a soft limit for two limit switches by using DigitalInput.
     *
     *  @param value for the left endpoint of the Turret
     *  @param value for the right endpoint of the Turret
     */
    void setSoftLimit(double forwardLimit, double reverseLimit);

    /**
     *  Resets the encoder wired to the Turret.
     */
    void resetEncoder();

    /**
     *  Returns whether is the controller is enabled or disabled.
     *
     *  @return bool if controller is enabled
     */
    bool IsControllerEnabled() {}

    /**
     *  Resets everything in the subsystem to its default state by
     *  using each correspoding reset statement it gives.
     */
    void Reset();

    /**
     *  Sets the state for the Turret to execute.
     *
     *  This function is designed to be used in a state machine, using a
     * structure which clearly defines what is executed in each state called by
     * a certain subsystem. These states can be access from the Turret subsystem
     * header file.
     *
     *  @param State parameter from the State enum in the Turret header file
     */
    void setState(State state);

    /**
     *  Gets the current state in a state machine.
     *
     *  This function is designed to be used in a state machine, using a
     * structure which clearly defines what is executed in each state called by
     * a certain subsystem. These states can be access from the Turret subsystem
     * header file.
     */
    State getState();

    /**
     *  Gets encoder values from the encoder wired to the Turret.
     */
    double getEncoder() const;

private:
    frc::Notifier m_notifier;

    // Encoder
    frc::Encoder m_TurretEncoder{kTurretEncoderA, kTurretEncoderB};

    // Both hall sensors for each boundary of the turret
    frc::DigitalInput* m_RightHallSensor;
    frc::DigitalInput* m_LeftHallSensor;

    // Prevents motor from rotating past these encoder values
    double m_forwardLimit = std::numeric_limits<double>::infinity();
    double m_reverseLimit = -std::numeric_limits<double>::infinity();

    // Spark Max
    rev::CANSparkMax m_TurretSparkMax{kTurretSparkMaxPort,
                                      rev::CANSparkMax::MotorType::kBrushless};
    };
}  // namespace frc3512
