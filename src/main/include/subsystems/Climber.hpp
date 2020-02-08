// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/Joystick.h>
#include <frc/PowerDistributionPanel.h>
#include <rev/CANSparkMax.h>

#include "Constants.hpp"
#include "communications/PublishNode.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

class Climber : public SubsystemBase, public PublishNode {
public:
    enum class TransverserState { kLeft, kRight, kIdle };
    
    Climber();

    void SetTransverser(TransverserState transverserState);

    void ElevatorAscend();

    void ElevatorDescend();

    void ElevatorIdle();

    void ProcessMessage(const ButtonPacket& message);

private:
    rev::CANSparkMax m_armL{Constants::Climber::kArmPortL,
                            rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_armR{Constants::Climber::kArmPortR,
                            rev::CANSparkMax::MotorType::kBrushless};

    rev::CANSparkMax m_transverser{Constants::Climber::kTransverserPort,
                                   rev::CANSparkMax::MotorType::kBrushless};

    rev::CANSparkMax m_winch{Constants::Climber::kWinchPort,
                             rev::CANSparkMax::MotorType::kBrushless};
};

}  // namespace frc3512
