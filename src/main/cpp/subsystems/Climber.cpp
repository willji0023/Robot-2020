// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/Climber.hpp"

#include <frc/DriverStation.h>
#include <wpi/raw_ostream.h>

using namespace frc3512;
using namespace frc3512::Constants::Climber;

Climber::Climber() : PublishNode("Climber") {}

void Climber::SetTransverser(TransverserState transverserState) {
    if (transverserState == TransverserState::kLeft) {
        m_transverser.Set(kMotorForward);
    } else if (transverserState == TransverserState::kRight) {
        m_transverser.Set(kMotorReverse);
    } else {
        m_transverser.Set(kMotorIdle);
    }
}

// Note: Set values are not yet fixed.
void Climber::ElevatorAscend() {
    m_armL.Set(1.0);
    m_armR.Set(1.0);
}

void Climber::ElevatorDescend() {
    m_armL.Set(-1.0);
    m_armR.Set(-1.0);
}

void Climber::ElevatorIdle() {
    m_armL.Set(0.0);
    m_armR.Set(0.0);
}

void Climber::ProcessMessage(const ButtonPacket& message) {
    if (message.topic == "Robot/AppendageStick" && message.button == 7 &&
        message.pressed) {
        m_winch.Set(1.0);
    } else if (message.topic == "Robot/AppendageStick" && message.button == 8 &&
               message.pressed) {
        m_winch.Set(-1.0);
    } else {
        m_winch.Set(0.0);
    }
    if (message.topic == "Robot/AppendageStick" && message.button == 9 &&
        message.pressed) {
        ElevatorAscend();
    } else if (message.topic == "Robot/AppendageStick" &&
               message.button == 10 && message.pressed) {
        ElevatorDescend();
    } else {
        ElevatorIdle();
    }
    if (message.topic == "Robot/AppendageStick" && message.button == 11 &&
        message.pressed) {
        SetTransverser(TransverserState::kLeft);
    } else if (message.topic == "Robot/AppendageStick" &&
               message.button == 12 && message.pressed) {
        SetTransverser(TransverserState::kRight);
    } else if (message.topic == "Robot/AppendageStick" &&
               message.button == 11 && !message.pressed) {
        SetTransverser(TransverserState::kIdle);
    } else if (message.topic == "Robot/AppendageStick" &&
               message.button == 12 && !message.pressed) {
        SetTransverser(TransverserState::kIdle);
    }
}
