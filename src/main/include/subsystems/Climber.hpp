// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/Joystick.h>
#include <frc/PowerDistributionPanel.h>
#include <rev/CANSparkMax.h>

#include "Constants.hpp"
#include "Robot.hpp"
#include "communications/PublishNode.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

class Climber : public SubsystemBase, public PublishNode {
public:
    enum class TransverserState { kForward, kReverse, kIdle };
    /**
     * Constructs a Climber.
     *
     * @param pdp The robot's power distribution panel.
     */
    explicit Climber(frc::PowerDistributionPanel& pdp);

    void SetTransverser(TransverserState transverserState);

    void ElevatorAscend();

    void ElevatorDescend();

    void ElevatorIdle();

    void SubsystemPeriodic();

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

    frc::PowerDistributionPanel& m_pdp;
};

}  // namespace frc3512
