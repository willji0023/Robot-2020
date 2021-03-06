// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include <fstream>

#include <frc/system/plant/DCMotor.h>
#include <gtest/gtest.h>
#include <mockdata/RoboRioData.h>
#include <units/units.h>
#include <wpi/circular_buffer.h>

#include "Constants.hpp"
#include "controllers/FlywheelController.hpp"

TEST(FlywheelControllerTest, ReachesGoal) {
    using frc3512::Constants::kDt;
    using frc3512::Constants::Flywheel::kGearRatio;
    using frc3512::Constants::Flywheel::kMaxAngularVelocity;

    frc3512::FlywheelController controller{{80.0}, {12.0}, kDt};
    controller.Reset();
    controller.Enable();

    controller.SetMeasuredAngularVelocity(0_rad_per_s);
    controller.SetGoal(500.0_rad_per_s);

    Eigen::Matrix<double, 1, 1> trueXhat = Eigen::Matrix<double, 1, 1>::Zero();

    auto currentTime = 0_s;
    while (currentTime < 10_s) {
        auto dt = kDt + units::second_t{frc::MakeWhiteNoiseVector(0.001)(0, 0)};
        Eigen::Matrix<double, 1, 1> noise =
            frc::MakeWhiteNoiseVector<1>({0.08});

        controller.SetMeasuredAngularVelocity(
            units::radians_per_second_t{trueXhat(0) + noise(0, 0)});

        controller.Update(kDt, currentTime);
        currentTime += dt;

        constexpr auto Vbat = 12_V;
        constexpr auto Rbat = 0.03_Ohm;
        Eigen::Matrix<double, 1, 1> u;
        u << controller.ControllerVoltage().to<double>();

        // Account for battery voltage drop due to current draw
        constexpr auto motors = frc::DCMotor::NEO(2);
        units::ampere_t load = motors.Current(
            units::radians_per_second_t{trueXhat(0, 0) / kGearRatio},
            units::volt_t{u(0, 0)});
        units::volt_t vLoaded = Vbat - load * Rbat - load * Rbat;
        double dsVoltage =
            vLoaded.to<double>() + frc::MakeWhiteNoiseVector(0.1)(0, 0);
        HALSIM_SetRoboRioVInVoltage(0, dsVoltage);
        Eigen::Matrix<double, 1, 1> trueU = u;
        trueU *= dsVoltage / 12.0;

        trueXhat(0) = controller.EstimatedAngularVelocity().to<double>();
    }
    EXPECT_TRUE(controller.AtGoal());
}
