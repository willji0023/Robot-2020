// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/AnalogInput.h>
#include <frc/Solenoid.h>
#include <rev/CANSparkMax.h>

#include "Constants.hpp"
#include "communications/PublishNode.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

enum class State {
    kIdle,
    // exit state when intake runs
    kDropBalls,
    // conveyor and intake are on, reverse motor so that conveyor drops balls to
    // bottom
    kRaiseBalls,
    // when hopper is full, run conveyor motor to bring balls back up
};

class Intake : public SubsystemBase, public PublishNode {
public:
    enum class ArmMotorState { kIntake, kOuttake, kIdle };

    Intake();

    void Deploy();

    void Stow();

    bool IsDeployed() const;

    void SetArmMotor(ArmMotorState armMotorState);

    void FeedBalls();
    /**
     * Returns whether or not the upper proximity sensor detects something; true
     * means something is detected, false means it is not.
     */
    bool IsUpperEnabled() const;
    /**
     * Returns whether or not the lower proximity sensor detects something.
     */
    bool IsLowerEnabled() const;

    void ProcessMessage(const ButtonPacket& message);

    void SubsystemPeriodic();

private:
    State m_state = State::kIdle;

    rev::CANSparkMax m_funnelMotor{Constants::Intake::kFunnelPort,
                                   rev::CANSparkMax::MotorType::kBrushless};

    rev::CANSparkMax m_conveyorMotor{Constants::Intake::kConveyorPort,
                                     rev::CANSparkMax::MotorType::kBrushless};

    rev::CANSparkMax m_armMotor{Constants::Intake::kArmMotorPort,
                                rev::CANSparkMax::MotorType::kBrushless};

    frc::AnalogInput m_upperSensor{Constants::Intake::kUpperAnalogPort};
    frc::AnalogInput m_lowerSensor{Constants::Intake::kLowerAnalogPort};

    frc::Solenoid m_arm{Constants::Intake::kArmPort};

    ButtonPacket m_buttonPacket;
};
}  // namespace frc3512
