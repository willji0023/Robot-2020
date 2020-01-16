// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#include "controllers/DrivetrainController.hpp"
#include "controllers/TurretController.hpp"

#include <cmath>

#include <frc/system/NumericalJacobian.h>
#include <frc/system/RungeKutta.h>

using namespace frc3512;
using namespace frc3512::Constants::Turret;

TurretController::TurretController() { 
    m_y.setZero();
}

void TurretController::Enable() { m_loop.Enable(); }

void TurretController::Disable() { m_loop.Disable(); }

void TurretController::SetGoal(double goal) {
    m_angleProfile = frc::TrapezoidProfile<units::radians>{
        constraints,
        {units::radian_t{goal}, 0_rad_per_s},
        {units::radian_t{EstimatedAngle()}, 0_rad_per_s}};
    m_goal = {units::radian_t{goal}, 0_rad_per_s};
}

void TurretController::SetReferences(
    units::radian_t angle, units::radians_per_second_t velocity) {
    double angleRef = units::unit_cast<double>(angle);
    double velocityRef = units::unit_cast<double>(velocity);
    Eigen::Matrix<double, 2, 1> nextR;
    nextR << angleRef, velocityRef;
    m_loop.SetNextR(nextR);
}

bool TurretController::AtReferences() const { return m_atReferences; }

bool TurretController::AtGoal() const {
    return m_atReferences && m_goal == m_profiledReference;
}

void TurretController::SetMeasuredAngle(double measuredAngle) {
    m_y(0, 0) = measuredAngle;
}

double TurretController::ControllerVoltage() const {
    if (!m_climbing) {
        return m_loop.U(0);
    } else {
        // Feedforward compensates for unmodeled extra weight from lifting robot
        // while climbing
        return m_loop.U(0) - 2.0;
    }
}

void TurretController::SetClimbing(bool climbing) {
    m_climbing = climbing;
}

units::radian_t TurretController::EstimatedAngle() const { return units::radian_t(m_loop.Xhat(0)); }

double TurretController::EstimatedAngularVelocity() const {
    return m_loop.Xhat(1);
}

double TurretController::AngleError() const {
    return m_loop.Error()(0, 0);
}

double TurretController::AngularVelocityError() const {
    return m_loop.Error()(1, 0);
}

double TurretController::AngleReference() {
    return m_profiledReference.position.to<double>();
}

double TurretController::AngularVelocityReference() {
    return m_profiledReference.velocity.to<double>();
}

units::radian_t TurretController::CalculateTheta(Eigen::Vector2d target, Eigen::Vector2d turret) {
    return units::math::atan2(target(1) - turret(1), target(0) - turret(0));
}

units::radians_per_second_t TurretController::CalculateOmega(Eigen::Vector2d v, Eigen::Vector2d r) {
    return units::radians_per_second_t((v - v.dot(r) / r.dot(r) * r).norm() / r.norm());
}

void TurretController::Update(units::second_t dt) {
    // Calculate next drivetrain and turret pose in global frame
    frc::Transform2d turretPoseInDrivetrainToGlobal{frc::Pose2d(kTx, kTy, 0_rad), drivetrainPoseInGlobal};
    auto drivetrainNextXhat = frc::RungeKutta(&DrivetrainController::Dynamics, drivetrainXhat, drivetrainU, dt);
    frc::Pose2d drivetrainNextPoseInGlobal{units::meter_t(drivetrainNextXhat(0)), units::meter_t(drivetrainNextXhat(1)), units::radian_t(drivetrainNextXhat(2))};
    auto plant = &m_loop.Plant();
    auto turretNextXhat{plant->CalculateX(m_loop.Xhat(), m_loop.Controller().U(), dt)};
    frc::Pose2d turretNextPoseInLocal{0_m, 0_m, units::radian_t(turretNextXhat(1))}; 
    auto turretNextPoseInGlobal = turretNextPoseInLocal.TransformBy(turretPoseInDrivetrainToGlobal);

    // Find angle reference for this timestep
    auto turretThetaToTargetInGlobal = CalculateTheta(ToVector2d(targetPose.Translation()), ToVector2d(turretNextPoseInGlobal.Translation()));
    auto drivetrainThetaInGlobal = drivetrainNextPoseInGlobal.Rotation().Radians();
    auto turretDesiredThetaInDrivetrain = turretThetaToTargetInGlobal - drivetrainThetaInGlobal;

    // Find angular velocity reference for this timestep
    Eigen::Matrix<double, 2, 1> turretVelocityVectorInGlobal;
    double drivetrainVelocity = (drivetrainNextXhat(3) + drivetrainNextXhat(4)) / 2.0;
    turretVelocityVectorInGlobal <<  drivetrainVelocity * units::math::cos(drivetrainNextXhat(2)), drivetrainVelocity * units::math::sin(drivetrainNextXhat(2));
    Eigen::Matrix<double, 2, 1> turretDisplacementVectorInGlobal = ToVector2d(targetPose.Translation() - turretNextPoseInGlobal.Translation());
    auto turretOmegaToTargetInGlobal = CalculateOmega(turretVelocityVectorInGlobal, turretDisplacementVectorInGlobal);
    auto drivetrainOmegaInGlobal = drivetrainNextXhat(4) - drivetrainNextXhat(3) / Constants::Drivetrain::kWheelbaseWidth;
    auto turretDesiredOmegaInDrivetrain = turretOmegaToTargetInGlobal - drivetrainOmegaInGlobal;

    turretLogger.Log(m_y(0, 0), EstimatedAngle(), AngleReference(), ControllerVoltage(),
                       EstimatedAngularVelocity(), AngularVelocityReference());

    SetReferences(turretDesiredThetaInDrivetrain, turretDesiredOmegaInDrivetrain);

    m_loop.Correct(m_y);

    auto error = m_loop.Error();
    m_atReferences = std::abs(error(0, 0)) < kAngleTolerance &&
                     std::abs(error(1, 0)) < kAngularVelocityTolerance;

    m_loop.Predict(dt);
}

void TurretController::Reset() { m_loop.Reset(); }

Eigen::Vector2d ToVector2d(frc::Translation2d translation) {
    Eigen::Vector2d result;
    result << translation.X().to<double>(), translation.Y().to<double>();
    return result;
}