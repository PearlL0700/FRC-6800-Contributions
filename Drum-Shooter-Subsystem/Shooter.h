#pragma once

#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Transform3d.h"
#include "frc/geometry/Pose2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/length.h"
#include "units/velocity.h"
#include "valkyrie/BaseSubsystem.h"
#include "valkyrie/controllers/PhoenixController.h"
#include <valkyrie/controllers/BaseController.h>
#include "constants/ShooterConstants.h"
#include "valkyrie/controllers/PIDF.h"
#include "valkyrie/sensors/CurrentSensor.h"

#include <wpi/interpolating_map.h>

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc/simulation/DCMotorSim.h>
#include <cmath>
#include <vector>

#include "valkyrie/Gamepad.h"
#include <frc/Alert.h>

#include "Drivetrain.h"

struct ShotParameter {
    units::turns_per_second_t flyWheelVelocity;
    units::turn_t hoodAngle;

    ShotParameter Interpolate(const ShotParameter& endValue, double t) const {
        return {flyWheelVelocity + (endValue.flyWheelVelocity - flyWheelVelocity) * t, hoodAngle + (endValue.hoodAngle - hoodAngle) * t};
    }
    ShotParameter operator+(const ShotParameter& other) const {
        return {flyWheelVelocity + other.flyWheelVelocity, hoodAngle + other.hoodAngle};
    }

    ShotParameter operator*(double scalar) const { return {flyWheelVelocity * scalar, hoodAngle * scalar}; }

    friend ShotParameter operator*(double scalar, const ShotParameter& param) { return param * scalar; }
};

class Shooter : public valor::BaseSubsystem {
   public:
    Shooter(frc::TimedRobot& robot, Drivetrain& drivetrain, valor::CANdleSensor& leds);
    ~Shooter();

    void init();
    void resetState();
    void assessInputs() override;
    void analyzeDashboard() override;
    void assignOutputs() override;
    void SimulationPeriodic() override;

    void calculateSetpointsSOTM(frc::Translation3d goal, frc::Pose2d predictedPose, frc::Pose2d actualPose, frc::ChassisSpeeds speeds);
    units::degree_t calculateHoodAngle(units::length::meter_t distance);
    units::turns_per_second_t calculateFlywheelSpeed(units::length::meter_t distance);

    bool isSystemReady();
    bool isFlywheelReady(units::turns_per_second_t flywheelTolerance);
    bool isHoodReady(units::degree_t hoodtolerance);

    frc::Pose3d getWorldShooterPose();
    frc::Pose3d getWorldHoodPose();
    frc::Pose2d getHubPose();
    frc::Pose2d getPresetShuttleZone();
    frc::Pose2d getShuttlePose();

    bool shuttleOnSameSide();
    bool inAllianceZone(frc::Pose2d pose);

    wpi::interpolating_map<units::length::meter_t, ShotParameter> getInterpolation();

    std::vector<frc::Pose3d> simulateShot(units::meters_per_second_t launchVel, frc::Pose2d turretPose, units::turn_t hoodAngle,
                                          units::turn_t turretAngle, units::meter_t launchHeight);
    std::vector<frc::Pose3d> simulateShot(units::meters_per_second_t launchVel, frc::Pose2d turretPose, units::turn_t hoodAngle,
                                          units::turn_t turretAngle, units::meter_t launchHeight,
                                          frc::Translation2d vel);  // TODO: Switch to chassis speeds

    void setManualFlywheelSpeed(units::turns_per_second_t setpoint);
    void setManualHoodAngle(units::turn_t setpoint);
    void enableAutoShooting();
    void setNextPosition(frc::Pose3d pose);
    void setNextVelocity(frc::Translation2d vels);
    void makeHubShoot();
    void makeShuttle();
    units::meters_per_second_t convertTpsToMps(units::turns_per_second_t tps, units::meter_t dist);
    units::turns_per_second_t convertMpsToTps(units::meters_per_second_t mps, units::meter_t dist);
    units::turns_per_second_t getFlywheelTarget();

    frc2::CommandPtr hoodPitSequence();
    frc2::CommandPtr flywheelPitSequence();

    enum class FLYWHEEL_STATE { DISABLE, MANUAL, SHOOT, PITMODE };

    enum class HOOD_STATE { DISABLE, MANUAL, TRACKING, PITMODE, HOME };

    struct x {
        FLYWHEEL_STATE flywheelState;
        HOOD_STATE hoodState;

        units::turns_per_second_t flywheelTarget;
        units::degree_t hoodTarget;
        frc::Rotation2d projectileAngle, previousProjectileAngle;
        units::turns_per_second_t turretTargetVelocity;

        frc::Pose2d trackingTarget;

        units::meter_t distanceToGoal;

        units::meters_per_second_t velTarget;
        bool shootHub;

        frc::Pose3d nextPosition;
        frc::Translation2d nextVelocity;
        frc::Pose2d shuttlePose;
        double velocityLoss = 0.77f;
        units::degree_t projectileAngleVariation = 0.0_deg;
    } state;

    struct ManualMode {
        units::turn_t hoodAngle;
        units::turn_t turretAngle;
        units::turns_per_second_t flywheelSpeed;
    } manual;

    bool pitMode;
    frc::Pose2d predictedPose;

   private:
    valor::PhoenixController<ctre::phoenix6::controls::VelocityTorqueCurrentFOC> leftFlywheelMotor;
    valor::PhoenixController<ctre::phoenix6::controls::VelocityTorqueCurrentFOC> rightOneFlywheelMotor;
    valor::PhoenixController<ctre::phoenix6::controls::VelocityTorqueCurrentFOC> rightTwoFlywheelMotor;
    valor::PhoenixController<ctre::phoenix6::controls::VelocityVoltage, ctre::phoenix6::controls::PositionTorqueCurrentFOC> hoodMotor;

    frc::sim::DCMotorSim leftFlywheelMotorSim;
    frc::sim::DCMotorSim rightOneFlywheelMotorSim;
    frc::sim::DCMotorSim rightTwoFlywheelMotorSim;
    frc::sim::DCMotorSim hoodMotorSim;

    frc::Pose3d calculateHoodPosition(units::radian_t);

    Drivetrain& drivetrain;
    valor::CANdleSensor& leds;

    std::bitset<2> systemTolerances;

    wpi::interpolating_map<units::length::meter_t, ShotParameter> hubInterpolation;
    wpi::interpolating_map<units::length::meter_t, ShotParameter> shuttleInterpolation;
    frc::LinearFilter<units::degree_t> turretAngleFilter = frc::LinearFilter<units::degree_t>::MovingAverage(7);

    frc::Alert hoodStatus{"", frc::Alert::AlertType::kInfo};
    frc::Alert flywheelStatus{"", frc::Alert::AlertType::kInfo};
};
