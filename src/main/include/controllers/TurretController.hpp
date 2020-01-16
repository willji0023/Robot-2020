// Copyright (c) 2018-2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <Eigen/Core>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/logging/CSVLogFile.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform2d.h>  
#include <frc/system/LinearSystem.h>
#include <frc/system/LinearSystemLoop.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/SingleJointedArmSystem.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/units.h>

#include "Constants.hpp"

namespace frc3512 {

class TurretController {
public:
    // State tolerances in radians and radians/sec respectively.
    static constexpr double kAngleTolerance = 0.05;
    static constexpr double kAngularVelocityTolerance = 2.0;

    TurretController();

    TurretController(const TurretController&) = delete;
    TurretController& operator=(const TurretController&) = delete;

    /**
     * Enables the control loop.
     */
    void Enable();

    /**
     * Disables the control loop.
     */
    void Disable();

    /**
     * Sets the end goal of the controller profile.
     *
     * @param goal Position in radians to set the goal to.
     */
    void SetGoal(double goal);

    /**
     * Sets the references.
     *
     * @param angle  Angle of the carriage in radians.
     * @param angularVelocity  Angular velocity of the carriage in radians per
     *                         second.
     */
    void SetReferences(units::radian_t angle,
                       units::radians_per_second_t angularVelocity);

    /**
     * Returns whether or not position and velocity are tracking the profile.
     */
    bool AtReferences() const;

    /**
     * Returns whether or not the goal has been reached.
     */
    bool AtGoal() const;

    /**
     * Sets the current encoder measurement.
     *
     * @param measuredAngle Angle of the carriage in radians.
     */
    void SetMeasuredAngle(double measuredAngle);

    /**
     * Returns the control loop calculated voltage.
     */
    double ControllerVoltage() const;

    /**
     * Informs the controller if to use the climbing feedforward.
     *
     * @param climbing Whether or not to use the climbing feedforward.
     */
    void SetClimbing(bool climbing);

    /**
     * Returns the estimated angle.
     */
    units::radian_t EstimatedAngle() const;

    /**
     * Returns the estimated angular velocity.
     */
    double EstimatedAngularVelocity() const;

    /**
     * Returns the error between the angle reference and the angle
     * estimate.
     */
    double AngleError() const;

    /**
     * Returns the error between the angular velocity reference and the angular
     * velocity estimate.
     */
    double AngularVelocityError() const;

    /**
     * Returns the current angle reference.
     */
    double AngleReference();

    /**
     * Returns the current angular velocity reference.
     */
    double AngularVelocityReference();

    /**
     * 
     */
    units::radian_t CalculateTheta(Eigen::Vector2d target, Eigen::Vector2d turret);

    /**
     * 
     */
    units::radians_per_second_t CalculateOmega(Eigen::Vector2d v, Eigen::Vector2d r);
    
    /**
     * Executes the control loop for a cycle.
     */
    void Update(units::second_t dt);

    /**
     * Resets any internal state.
     */
    void Reset();

private:
    // The current sensor measurement.
    Eigen::Matrix<double, 1, 1> m_y;
    frc::TrapezoidProfile<units::radians>::State m_goal;

    frc::TrapezoidProfile<units::radians>::Constraints constraints{
        Constants::Turret::kMaxV, Constants::Turret::kMaxA};
    frc::TrapezoidProfile<units::radians> m_angleProfile{constraints,
                                                         {0_rad, 0_rad_per_s}};

    frc::TrapezoidProfile<units::radians>::State m_profiledReference;

    // frc::LinearSystem<2, 1, 1> m_plant = frc::IdentifyPositionSystem(Constants::Turret::kV, Constants::Turret::kA);
    frc::LinearSystem<2, 1, 1> m_plant = [=] {
        constexpr auto motor = frc::DCMotor::NEO();

        // Arm moment of inertia
        constexpr auto J = 0.6975_kg_sq_m;

        // Gear ratio
        constexpr double G = 302.22;

        return frc::SingleJointedArmSystem(motor, J, G);
    }();

    frc::LinearQuadraticRegulator<2, 1> m_controller{
        m_plant, {0.01245, 0.109726}, {9.0}, Constants::kDt};
    frc::KalmanFilter<2, 1, 1> m_observer{
        m_plant, Constants::kDt, {0.21745, 0.28726}, {0.01}};
    frc::LinearSystemLoop<2, 1, 1> m_loop{m_plant, m_controller, m_observer};

    bool m_atReferences = false;
    bool m_climbing = false;

    frc::Translation2d m_T{kTx, kTy};
    frc::Pose2d drivetrainPoseInGlobal;
    frc::Pose2d predictedDrivetrainPose;
    frc::Pose2d targetPose;
    Eigen::Matrix<double, 10, 1> drivetrainXhat;
    Eigen::Matrix<double, 2, 1> drivetrainU;

    frc::CSVLogFile turretLogger{"Turret", "Pos (rad)", "EstPos (rad)",
                                   "PosRef (rad)",   "Voltage (V)",
                                   "EstAngVel (rad/s)", "AngVelRef (rad/s)"};

    Eigen::Vector2d ToVector2d(frc::Translation2d translation);
};

}  // namespace frc3512
