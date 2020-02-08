// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/Intake.hpp"

using namespace frc3512;
using namespace frc3512::Constants::Intake;

Intake::Intake() : PublishNode("Intake") {}

void Intake::Deploy() { m_arm.Set(true); }

void Intake::Stow() { m_arm.Set(false); }

bool Intake::IsDeployed() const { return m_arm.Get(); }
/**
 * At the moment, the armMotor set values are not fixed; neither are the
 * funnel's and conveyor's. NOTE: All three motors (arm's motor, funnel's motor,
 * conveyor's motor) are now set to fixed values.
 */
void Intake::SetArmMotor(ArmMotorState armMotorState) {
    if (armMotorState == ArmMotorState::kIntake) {
        m_armMotor.Set(kMotorForward);
    } else if (armMotorState == ArmMotorState::kOuttake) {
        m_armMotor.Set(kMotorReverse);
    } else {
        m_armMotor.Set(kMotorIdle);
    }
}

void Intake::FeedBalls() {
    m_armMotor.Set(kMotorReverse);
    m_funnelMotor.Set(kMotorReverse);
    m_conveyorMotor.Set(kMotorReverse);
}

bool Intake::IsUpperEnabled() const { return m_upperSensor.GetValue() == 0; }

bool Intake::IsLowerEnabled() const { return m_lowerSensor.GetValue() == 0; }

void Intake::ProcessMessage(const ButtonPacket& message) {
    if (message.topic == "Robot/AppendageStick" && message.button == 4 &&
        message.pressed) {
        SetArmMotor(ArmMotorState::kIntake);
    } else if (message.topic == "Robot/AppendageStick" && message.button == 6 &&
               message.pressed) {
        SetArmMotor(ArmMotorState::kOuttake);
    } else if (message.topic == "Robot/AppendageStick" && message.button == 4 &&
               !message.pressed) {
        SetArmMotor(ArmMotorState::kIdle);
    } else if (message.topic == "Robot/AppendageStick" && message.button == 6 &&
               !message.pressed) {
        SetArmMotor(ArmMotorState::kIdle);
    }
    if (message.topic == "Robot/AppendageStick" && message.button == 3 &&
        message.pressed) {
        if (IsDeployed()) {
            Stow();
        } else if (!IsDeployed()) {
            Deploy();
        }
    }
}

void Intake::SubsystemPeriodic() {
    switch (m_state) {
        case State::kIdle: {
            if (m_armMotor.Get() > 0.5) {
                m_funnelMotor.Set(kMotorForward);
                m_conveyorMotor.Set(kMotorReverse);
                m_state = State::kDropBalls;
            }
            break;
        }
        case State::kDropBalls: {
            if (IsLowerEnabled() && IsUpperEnabled())
                m_conveyorMotor.Set(kMotorForward);
            m_funnelMotor.Set(kMotorIdle);
            m_state = State::kRaiseBalls;
            break;
        }
        case State::kRaiseBalls: {
            if (IsUpperEnabled()) m_conveyorMotor.Set(kMotorIdle);
            m_state = State::kIdle;
            break;
        }
    }
}
