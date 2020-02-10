// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/Turret.hpp"

using namespace frc3512;
using namespace frc3512::Constants::Turret;

void Turret::setVelocity(double velocity) { m_TurretSparkMax.Set(velocity); }

double Turret::getEncoder() const { m_TurretEncoder.Get(); }

void Turret::EnableController() {}

void Turret::DisableController() {}

bool Turret::IsControllerEnabled() {}

void Turret::Reset() {}

void Turret::resetEncoder() { m_TurretEncoder.Reset(); }

void Turret::setHardLimit(frc::DigitalInput* rightHallSensor,
                          frc::DigitalInput* leftHallSensor) {
    m_RightHallSensor = rightHallSensor;
    m_LeftHallSensor = leftHallSensor;
}

void Turret::setSoftLimit(double forwardLimit, double reverseLimit) {
    m_forwardLimit = forwardLimit;
    m_reverseLimit = reverseLimit;
}

void Turret::ProcessMessage(const ButtonPacket& message) {
    if(message.topic == "Robot/AppendageStick" && message.button == 6 &&
        message.pressed){
            
    } else if (message.topic == "Robot/AppendageStick" && message.button == 6 &&
        message.pressed){
            
    } else if (message.topic == "Robot/AppendageStick" && message.button == 6 &&
        message.pressed){
            
    }
}
