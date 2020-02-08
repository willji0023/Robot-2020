// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/DigitalInput.h>
#include <frc/Encoder.h>
#include <frc/Notifier.h>
#include <rev/CANSparkMax.h>

#include "Constants.hpp"
#include "communications/PublishNode.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

enum State { kIDLE = 0, kLeft, KRight, kDisabled };

class Turret : public SubsystemBase, public PublishNode {
public:
    Turret();
    Turret(const Turret&) = delete;
    Turret& operator=(const Turret&) = delete;

    void setVelocity(double velocity) {}

    void EnableController() {}

    void DisableController() {}

    void resetEncoder();

    bool IsControllerEnabled() {}

    void Reset();

    void setState(State state);

    double getEncoder() const;

private:
    frc::Notifier m_notifier;

    frc::Encoder m_TurretEncoder{kTurretEncoderA, kTurretEncoderB};

    frc::DigitalInput m_RightHallSensor{kRightHallSensorPort};
    frc::DigitalInput m_LeftHallSensor{kLeftHallSensorPort};

    rev::CANSparkMax m_TurretSparkMax{kTurretSparkMaxPort,
                                      rev::CANSparkMax::MotorType::kBrushless};
};
}  // namespace frc3512
