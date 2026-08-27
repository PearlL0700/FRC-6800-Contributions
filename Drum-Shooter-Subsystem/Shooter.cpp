#include "subsystems/Shooter.h"
#include <algorithm>
#include <cstdio>
#include <iostream>
#include <numbers>
#include <vector>
#include <math.h>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <frc/system/plant/LinearSystemId.h>

#include "constants/Constants.h"
#include "constants/ShooterConstants.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Rotation3d.h"
#include "frc/geometry/Transform3d.h"
#include "units/angle.h"
#include "units/length.h"

#include "units/math.h"
#include "units/time.h"
#include "units/voltage.h"
#include "Drivetrain.h"
#include "valkyrie/drivetrain/Swerve.h"

#include <pathplanner/lib/auto/NamedCommands.h>
#include <frc/DriverStation.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <valkyrie/BaseSubsystem.h>
#include "valkyrie/controllers/BaseController.h"
#include <valkyrie/controllers/PhoenixController.h>

#define LATENCY_COMP 250_ms
#define FLYWHEEL_FOLLOWER_FREQUENCY 500_Hz

#define SHUTTLE_FLYWHEEL_TOLERANCE 15.0_tps
#define SHUTTLE_HOOD_TOLERANCE 8.0_deg
#define SHUTTLE_TURRET_TOLERANCE 7.0_deg

#define SHOOT_FLYWHEEL_TOLERANCE 5.0_tps
#define SHOOT_HOOD_TOLERANCE 4.0_deg
#define SHOOT_TURRET_TOLERANCE 5.0_deg

#define HOOD_HOME_POS 0.005_tr

#define G 9.81f
#define RHO 1.204f
#define CD 0.47f
#define R 0.15f
#define M 0.203f
#define DT 0.01f
#define CL 0.5f
#define BACKSPIN_SPEED 1.5f

// Position error -> velocity P gain for turret velocity tracking (turns/s per turn of error)
#define TURRET_TRACKING_KP 12.0

static constexpr units::length::foot_t PHYSICAL_TO_ROBOT_TRANSLATION = 2.917_ft;
static constexpr units::length::foot_t ARBITRARY_OFFSET_FOR_FAR_SHOTS = -5_in;
static constexpr units::turns_per_second_t ARBITRARY_OVERSHOOT_FOR_FAR_SHOTS = 1.2_tps;

constexpr units::meter_t FLYWHEEL_RADIUS = 0.0508_m;
constexpr units::millisecond_t DELTA_TIME = 350_ms;

Shooter::Shooter(frc::TimedRobot& _robot, Drivetrain& _drivetrain, valor::CANdleSensor& _leds)
      : valor::BaseSubsystem("Shooter"),
        leftFlywheelMotor(valor::PhoenixControllerType::KRAKEN_X60_FOC, CANIDs::FLYWHEEL_LEFT, valor::NeutralMode::Coast,
                          Constants::Shooter::flywheelMotorInverted(), "baseCAN"),
        rightOneFlywheelMotor(valor::PhoenixControllerType::KRAKEN_X60_FOC, CANIDs::FLYWHEEL_RIGHT_ONE, valor::NeutralMode::Coast,
                              !Constants::Shooter::flywheelMotorInverted(), "baseCAN"),
        rightTwoFlywheelMotor(valor::PhoenixControllerType::KRAKEN_X60_FOC, CANIDs::FLYWHEEL_RIGHT_TWO, valor::NeutralMode::Coast,
                              !Constants::Shooter::flywheelMotorInverted(), "baseCAN"),
        hoodMotor(valor::PhoenixControllerType::KRAKEN_X44_FOC, CANIDs::HOOD, valor::NeutralMode::Brake,
                  Constants::Shooter::hoodMotorInverted(), "baseCAN"),
        leftFlywheelMotorSim{
            frc::LinearSystemId::DCMotorSystem(leftFlywheelMotor.motorSpec, 0.0001_kg_sq_m, Constants::Shooter::getFlywheelSensorToMech()),
            leftFlywheelMotor.motorSpec,
        },
        rightOneFlywheelMotorSim{
            frc::LinearSystemId::DCMotorSystem(rightOneFlywheelMotor.motorSpec, 0.0001_kg_sq_m,
                                               Constants::Shooter::getFlywheelSensorToMech()),
            rightOneFlywheelMotor.motorSpec,
        },
        rightTwoFlywheelMotorSim{
            frc::LinearSystemId::DCMotorSystem(rightTwoFlywheelMotor.motorSpec, 0.0001_kg_sq_m,
                                               Constants::Shooter::getFlywheelSensorToMech()),
            rightTwoFlywheelMotor.motorSpec,
        },
        hoodMotorSim{
            frc::LinearSystemId::DCMotorSystem(hoodMotor.motorSpec, 0.0001_kg_sq_m,
                                               Constants::Shooter::getHoodMotorToSensor() * Constants::Shooter::getHoodSensorToMech()),
            hoodMotor.motorSpec},
        drivetrain(_drivetrain),
        leds{_leds} {
    init();
}

Shooter::~Shooter() {}

void Shooter::resetState() {
    manual.hoodAngle = HOOD_HOME_POS;
    manual.flywheelSpeed = 0.0_tps;

    state.flywheelState = FLYWHEEL_STATE::DISABLE;
    state.hoodState = HOOD_STATE::DISABLE;

    state.hoodTarget = 0.0_tr;
    state.projectileAngle = 0.0_tr;
    state.flywheelTarget = 0.0_tps;

    state.nextPosition = frc::Pose3d{0.0_m, 0.0_m, 0_m, frc::Rotation3d(0_deg)};
    state.nextVelocity = frc::Translation2d{0.0_m, 0.0_m};

    state.shootHub = false;
    pitMode = false;

    predictedPose = drivetrain.cachedPredictedPose;
}

void Shooter::init() {
    // Flywheel setup

    for (auto& flywheelMotor : {&leftFlywheelMotor, &rightOneFlywheelMotor, &rightTwoFlywheelMotor}) {
        flywheelMotor->SetPeriodicLevel(valor::LogLevel::Important);
        flywheelMotor->setGearRatios(1.0, Constants::Shooter::getFlywheelSensorToMech());

        valor::PIDF flywheelPID = Constants::Shooter::getFlywheelPIDF();
        flywheelPID.maxVelocity = flywheelMotor->getMaxMechSpeed();
        flywheelPID.maxAcceleration = flywheelMotor->getMaxMechSpeed() / 1_s;
        flywheelMotor->setCurrentLimits(45_A, 60_A, 65_A, 1_s);
        flywheelMotor->setPIDF(flywheelPID);
        flywheelMotor->applyConfig();
    }

    // Hood setup
    hoodMotor.setGearRatios(Constants::Shooter::getHoodMotorToSensor(), Constants::Shooter::getHoodSensorToMech());
    hoodMotor.enableFOC(true);
    hoodMotor.setForwardLimit(Constants::Shooter::getHoodForwardLimit());
    hoodMotor.setReverseLimit(Constants::Shooter::getHoodReverseLimit());

    valor::PIDF hoodPID = Constants::Shooter::getHoodPIDF();
    hoodMotor.setPIDF(hoodPID);
    hoodMotor.setupCANCoder(CANIDs::HOOD_CAN, Constants::Shooter::getHoodMagnetOffset(),
                            ctre::phoenix6::signals::SensorDirectionValue::Clockwise_Positive, "baseCAN",
                            Constants::Shooter::getHoodDiscontinuityPoint());
    hoodMotor.setCurrentLimits(50_A, 60_A, 65_A, 1_s);
    hoodMotor.applyConfig();

    hubInterpolation.insert(1_ft + PHYSICAL_TO_ROBOT_TRANSLATION, {29.5_tps, 0.0_tr});
    hubInterpolation.insert(2_ft + PHYSICAL_TO_ROBOT_TRANSLATION, {29.5_tps, 0.01_tr});
    hubInterpolation.insert(3_ft + PHYSICAL_TO_ROBOT_TRANSLATION, {31_tps, 0.01_tr});
    hubInterpolation.insert(4_ft + PHYSICAL_TO_ROBOT_TRANSLATION, {32_tps, 0.01_tr});
    hubInterpolation.insert(5_ft + PHYSICAL_TO_ROBOT_TRANSLATION, {34_tps, 0.015_tr});
    hubInterpolation.insert(6_ft + PHYSICAL_TO_ROBOT_TRANSLATION, {36_tps, 0.015_tr});
    hubInterpolation.insert(8_ft + PHYSICAL_TO_ROBOT_TRANSLATION, {39_tps, 0.025_tr});
    hubInterpolation.insert(10_ft + PHYSICAL_TO_ROBOT_TRANSLATION, {46_tps, 0.0325_tr});
    hubInterpolation.insert(12_ft + PHYSICAL_TO_ROBOT_TRANSLATION, {52_tps, 0.0375_tr});
    hubInterpolation.insert(14_ft + PHYSICAL_TO_ROBOT_TRANSLATION, {59_tps, 0.04_tr});

    shuttleInterpolation.insert(10_ft, {25_tps, 0.09_tr});
    shuttleInterpolation.insert(12_ft, {28_tps, 0.09_tr});
    shuttleInterpolation.insert(14_ft, {31_tps, 0.09_tr});
    shuttleInterpolation.insert(16_ft, {38_tps, 0.05_tr});
    shuttleInterpolation.insert(18_ft, {43_tps, 0.05_tr});
    shuttleInterpolation.insert(20_ft, {49_tps, 0.05_tr});
    shuttleInterpolation.insert(22_ft, {60_tps, 0.06_tr});

    LogChild("Hood Motor", &hoodMotor);
    LogChild("Flywheel Left Motor", &leftFlywheelMotor);
    LogChild("Flywheel Right One Motor", &rightOneFlywheelMotor);
    LogChild("Flywheel Right Two Motor", &rightTwoFlywheelMotor);

    resetState();
}

void Shooter::assessInputs() {
    pitMode = ReadLog("Pit Mode", pitMode);
    if (pitMode) {
        state.hoodState = HOOD_STATE::PITMODE;
        state.flywheelState = FLYWHEEL_STATE::PITMODE;
    }

    if (frc::DriverStation::IsAutonomous()) {
        if (state.shootHub) {
            state.trackingTarget = getHubPose();
        } else {
            state.trackingTarget = getPresetShuttleZone();
        }
    } else {
        if (driverGamepad->rightTriggerActive() || driverGamepad->GetRightBumperButton() || driverGamepad->leftTriggerActive()) {
            state.trackingTarget = getHubPose();
        } else if (driverGamepad->GetLeftBumperButton()) {
            state.trackingTarget = getShuttlePose();
        }
    }
}

frc::Pose3d Shooter::calculateHoodPosition(units::radian_t shooterPosition) {
    frc::Pose3d hoodPose = frc::Pose3d{} + Constants::Shooter::hoodPosition();
    return frc::Pose3d{hoodPose.Translation(), {0_deg, hoodMotor.getPosition(), 0_deg}}.RotateAround(
        Constants::Shooter::shooterPosition().Translation(), {});
}

void Shooter::analyzeDashboard() {
    if (frc::DriverStation::IsTeleop()) {
        state.nextPosition = getWorldShooterPose();
        state.nextVelocity = frc::Translation2d{units::meter_t{drivetrain.cachedChassisFieldSpeed.vx.to<float>()},
                                                units::meter_t{drivetrain.cachedChassisFieldSpeed.vy.to<float>()}};
    }

    predictedPose = drivetrain.getPredictedPose(drivetrain.cachedCalculatedPose, ReadLog("Delta Time", DELTA_TIME));

    WriteLog("Predicted Pose", predictedPose);

    // calculateSetpointsSOTM(frc::Pose3d{state.trackingTarget}.Translation(), predictedPose, drivetrain.cachedCalculatedPose,
    //                        drivetrain.cachedChassisFieldSpeed);

    state.hoodTarget = calculateHoodAngle(state.trackingTarget.Translation().Distance(drivetrain.cachedCalculatedPose.Translation()));
    state.flywheelTarget =
        calculateFlywheelSpeed(state.trackingTarget.Translation().Distance(drivetrain.cachedCalculatedPose.Translation()));

    drivetrain.state.projectileAngle = (state.trackingTarget.Translation() - drivetrain.cachedCalculatedPose.Translation()).Angle();

    state.previousProjectileAngle = state.projectileAngle;

    WriteLog("Turret World Relative", getWorldShooterPose());
    WriteLog("Turret Robot Relative", frc::Pose3d(Constants::Shooter::shooterPosition().Translation(), frc::Rotation3d{}));

    WriteLog("Hood Robot Relative", calculateHoodPosition(Constants::Shooter::shooterPosition().Rotation().Z()));
    WriteLog("Hood World Relative", getWorldHoodPose());

    WriteLog("Current Target Pose", state.trackingTarget);

    WriteLog("Field Calibration/Shooter distance to Target",
             (getWorldShooterPose().ToPose2d() - state.trackingTarget).Translation().Norm());

    WriteLog("Hood State", state.hoodState);
    WriteLog("Flywheel State", state.flywheelState);

    // Run CANCoder magnet health checks at ~2Hz (driven by drivetrain counter)
    if (drivetrain.diagLoopCounter == 0) {
        leds.setLED(4, valor::CANdleSensor::cancoderMagnetHealthGetter(hoodMotor.getCANCoder()));
    }
}

void Shooter::assignOutputs() {
    if (state.flywheelState == FLYWHEEL_STATE::PITMODE) {
        leftFlywheelMotor.setSpeed(0_tps);
        rightOneFlywheelMotor.setSpeed(0_tps);
        rightTwoFlywheelMotor.setSpeed(0_tps);
    } else if (state.flywheelState == FLYWHEEL_STATE::SHOOT) {
        leftFlywheelMotor.setSpeed(state.flywheelTarget);
        rightOneFlywheelMotor.setSpeed(state.flywheelTarget);
        rightTwoFlywheelMotor.setSpeed(state.flywheelTarget);
    } else if (state.flywheelState == FLYWHEEL_STATE::MANUAL) {
        leftFlywheelMotor.setSpeed(manual.flywheelSpeed);
        rightOneFlywheelMotor.setSpeed(manual.flywheelSpeed);
        rightTwoFlywheelMotor.setSpeed(manual.flywheelSpeed);
    } else {
        leftFlywheelMotor.setPower(0_V);
        rightOneFlywheelMotor.setPower(0_V);
        rightTwoFlywheelMotor.setPower(0_V);
    }

    if (state.hoodState == HOOD_STATE::PITMODE) {
        hoodMotor.setPosition(HOOD_HOME_POS);
    } else if (state.hoodState == HOOD_STATE::TRACKING) {
        hoodMotor.setPosition(state.hoodTarget);
    } else if (state.hoodState == HOOD_STATE::HOME) {
        hoodMotor.setPosition(HOOD_HOME_POS);
    } else if (state.hoodState == HOOD_STATE::MANUAL) {
        hoodMotor.setPosition(manual.hoodAngle);
    } else {
        hoodMotor.setPower(0_V);
    }
}

void Shooter::SimulationPeriodic() {
    leftFlywheelMotorSim.SetInputVoltage(leftFlywheelMotor.calculateAppliedVoltage());
    rightOneFlywheelMotorSim.SetInputVoltage(rightOneFlywheelMotor.calculateAppliedVoltage());
    rightTwoFlywheelMotorSim.SetInputVoltage(rightTwoFlywheelMotor.calculateAppliedVoltage());
    hoodMotorSim.SetInputVoltage(hoodMotor.calculateAppliedVoltage());

    leftFlywheelMotorSim.Update(frc::TimedRobot::kDefaultPeriod);
    rightOneFlywheelMotorSim.Update(frc::TimedRobot::kDefaultPeriod);
    rightTwoFlywheelMotorSim.Update(frc::TimedRobot::kDefaultPeriod);
    hoodMotorSim.Update(frc::TimedRobot::kDefaultPeriod);

    leftFlywheelMotor.setSimState(leftFlywheelMotorSim);
    rightOneFlywheelMotor.setSimState(rightOneFlywheelMotorSim);
    rightTwoFlywheelMotor.setSimState(rightTwoFlywheelMotorSim);
    hoodMotor.setSimState(hoodMotorSim);
}

bool Shooter::isFlywheelReady(units::turns_per_second_t flywheelTolerance) {
    return frc::IsNear(state.flywheelTarget,
                       units::math::min(leftFlywheelMotor.getSpeed(),
                                        units::math::min(rightOneFlywheelMotor.getSpeed(), rightTwoFlywheelMotor.getSpeed())),
                       flywheelTolerance);
}

bool Shooter::isHoodReady(units::degree_t hoodTolerance) {
    return frc::IsNear(state.hoodTarget, hoodMotor.getPosition().convert<units::degree>(), hoodTolerance);
}

bool Shooter::isSystemReady() {
    systemTolerances[0] = isHoodReady(inAllianceZone(getWorldShooterPose().ToPose2d()) ? SHOOT_HOOD_TOLERANCE : SHUTTLE_HOOD_TOLERANCE);
    systemTolerances[1] =
        isFlywheelReady(inAllianceZone(getWorldShooterPose().ToPose2d()) ? SHOOT_FLYWHEEL_TOLERANCE : SHUTTLE_FLYWHEEL_TOLERANCE);
    WriteLog("Is System Ready", systemTolerances.to_string());
    return systemTolerances.all();
}

units::degree_t Shooter::calculateHoodAngle(units::length::meter_t distance) {
    return getInterpolation()[distance].hoodAngle - Constants::Shooter::getHoodDegOffset();
}

units::turns_per_second_t Shooter::calculateFlywheelSpeed(units::length::meter_t distance) {
    return getInterpolation()[distance].flyWheelVelocity;
}

void Shooter::calculateSetpointsSOTM(frc::Translation3d goal, frc::Pose2d predictedPose, frc::Pose2d actualPose,
                                     frc::ChassisSpeeds speeds) {
    // TODO: Refactor to be more abstract and less confusing.
    //      Why are we using turretPosition?
    //      Why not have predictedPose and actualPose be the world position of the system?
    //      ChassisSpeeds can be of the system as well
    //          Retype speeds to be a vector of speeds to be less confusing

    frc::Pose3d predictedPointPosition =
        frc::Pose3d{predictedPose} + frc::Transform3d{Constants::Shooter::shooterPosition().Translation(), frc::Rotation3d{}};

    frc::Translation3d predictedPointTranslation = predictedPointPosition.Translation();

    frc::Translation3d pointToGoal = goal - predictedPointTranslation;
    units::meter_t distance2d = pointToGoal.ToTranslation2d().Norm();

    ShotParameter params = getInterpolation()[distance2d];
    units::meters_per_second_t stationaryVel = convertTpsToMps(params.flyWheelVelocity, distance2d);
    frc::Rotation2d launchAngleStationary = 90_deg - params.hoodAngle;
    frc::Rotation2d turretAngleStationary = pointToGoal.ToTranslation2d().Angle();

    Eigen::Vector3d stationaryVector{launchAngleStationary.Cos() * turretAngleStationary.Cos(),
                                     launchAngleStationary.Cos() * turretAngleStationary.Sin(), launchAngleStationary.Sin()};

    stationaryVector *= stationaryVel.value();

    frc::Translation2d turretOffsetRobot = Constants::Shooter::shooterPosition().Translation().ToTranslation2d();
    frc::Translation2d turretOffsetField = turretOffsetRobot.RotateBy(predictedPose.Rotation());

    Eigen::Vector3d tangentialVelocity{-speeds.omega.value() * turretOffsetField.Y().value(),
                                       speeds.omega.value() * turretOffsetField.X().value(), 0};

    Eigen::Vector3d robotVector{speeds.vx.value(), speeds.vy.value(), 0};

    Eigen::Vector3d movingVector = stationaryVector - (robotVector + tangentialVelocity);

    units::meters_per_second_t movingVelocity{movingVector.norm()};
    units::meters_per_second_t horizontalNorm{movingVector.head<2>().norm()};
    units::radian_t projectileAngle =
        units::math::atan2(units::meters_per_second_t{movingVector.y()}, units::meters_per_second_t{movingVector.x()});
    units::radian_t hoodAngle = 90_deg - units::math::atan2(units::meters_per_second_t{movingVector.z()}, horizontalNorm);

    state.projectileAngle = projectileAngle;
    state.hoodTarget = hoodAngle - Constants::Shooter::getHoodDegOffset();
    state.flywheelTarget = convertMpsToTps(movingVelocity, 0_m);
}

frc::Pose3d Shooter::getWorldShooterPose() {
    frc::Pose2d robotPose = drivetrain.cachedCalculatedPose;
    frc::Pose3d turretOriginPosition =
        frc::Pose3d(robotPose) + frc::Transform3d{Constants::Shooter::shooterPosition().Translation(), frc::Rotation3d{}};

    return turretOriginPosition;
}

frc::Pose3d Shooter::getWorldHoodPose() {
    frc::Pose2d robotPose = drivetrain.cachedCalculatedPose;

    frc::Rotation3d hoodRotation{0_deg, state.hoodTarget + Constants::Shooter::getHoodDegOffset(),
                                 Constants::Shooter::shooterPosition().Rotation().Z()};

    frc::Pose3d hoodOriginPosition =
        frc::Pose3d(robotPose) + frc::Transform3d{Constants::Shooter::shooterPosition().Translation(), hoodRotation};

    return hoodOriginPosition;
}

frc::Pose2d Shooter::getHubPose() {
    frc::Pose2d hubPose = frc::Pose2d((frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed ? 1 : -1) *
                                          Constants::GameSpecific::CENTER_HUB_X,
                                      0_in, {}) +
                          Constants::GameSpecific::CENTER_FIELD_TO_WORLD_FRAME;

    return hubPose;
}

frc::Pose2d Shooter::getPresetShuttleZone() {
    frc::Pose2d shuttleOutpost = frc::Pose2d((frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed ? 1 : -1) *
                                                 Constants::GameSpecific::SHUTTLE_OUTPOST_X,
                                             Constants::GameSpecific::SHUTTLE_OUTPOST_Y, {}) +
                                 Constants::GameSpecific::CENTER_FIELD_TO_WORLD_FRAME;

    frc::Pose2d shuttleDepot = frc::Pose2d((frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed ? 1 : -1) *
                                               Constants::GameSpecific::SHUTTLE_DEPOT_X,
                                           Constants::GameSpecific::SHUTTLE_DEPOT_Y, {}) +
                               Constants::GameSpecific::CENTER_FIELD_TO_WORLD_FRAME;

    frc::Pose2d shuttleTracking =
        frc::Pose2d(drivetrain.cachedCalculatedPose.Nearest(std::vector<frc::Pose2d>{shuttleOutpost, shuttleDepot}));

    return shuttleTracking;
}

frc::Pose2d Shooter::getShuttlePose() {
    units::meter_t shuttleX = ReadLog("Shuttle Pose X", getPresetShuttleZone().X());
    units::meter_t dashboardPoseY = ReadLog("Shuttle Pose Y", getPresetShuttleZone().Y());

    // units::meter_t newYPose =
    //     dashboardPoseY > Constants::GameSpecific::HALF_FIELD_WIDTH ? Constants::GameSpecific::FIELD_WIDTH - dashboardPoseY :
    //     dashboardPoseY;

    // bool isCloseSide = getWorldTurretPose().ToPose2d().Y() < Constants::GameSpecific::HALF_FIELD_WIDTH;
    // units::meter_t shuttleY = isCloseSide ? newYPose : Constants::GameSpecific::FIELD_WIDTH - newYPose;

    return frc::Pose2d(shuttleX, dashboardPoseY, {});
}

bool Shooter::inAllianceZone(frc::Pose2d pose) {
    return frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue
        ? pose.X()<Constants::GameSpecific::ALLIANCE_ZONE_LENGTH : pose.X()>(Constants::GameSpecific::FIELD_LENGTH -
                                                                             Constants::GameSpecific::ALLIANCE_ZONE_LENGTH);
}

wpi::interpolating_map<units::length::meter_t, ShotParameter> Shooter::getInterpolation() {
    return inAllianceZone(getWorldShooterPose().ToPose2d()) ? hubInterpolation : shuttleInterpolation;
}

std::vector<frc::Pose3d> Shooter::simulateShot(units::meters_per_second_t launchVel, frc::Pose2d robotPose, units::turn_t hoodAngle,
                                               units::turn_t turretAngle, units::meter_t launchHeight) {
    std::vector<frc::Pose3d> trajectory;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> pos;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> vel;

    float vZ = launchVel.to<float>() * units::math::sin(hoodAngle);
    float vX = launchVel.to<float>() * units::math::cos(hoodAngle) * units::math::cos(turretAngle) +
               drivetrain.cachedChassisFieldSpeed.vx.to<float>();
    float vY = launchVel.to<float>() * units::math::cos(hoodAngle) * units::math::sin(turretAngle) +
               drivetrain.cachedChassisFieldSpeed.vy.to<float>();

    Eigen::Vector3d v(vX, vY, vZ);
    Eigen::Vector3d p(robotPose.X().to<float>(), robotPose.Y().to<float>(), launchHeight.to<float>());

    pos.push_back(p);
    vel.push_back(v);

    while (p[2] > 0) {
        auto v_dir = v / v.norm();
        auto drag_mag = 0.5 * RHO * 3.14 * std::pow(R, 2) * CD * std::pow(v.norm(), 2);

        Eigen::Vector3d drag = -(drag_mag * v_dir) / M;
        Eigen::Vector3d aG = Eigen::Vector3d(0, 0, -G);
        Eigen::Vector3d a = drag + aG;

        v = v + a * DT;
        p = p + v * DT;

        vel.push_back(v);
        pos.push_back(p);
    }

    for (auto position : pos) {
        trajectory.push_back(frc::Pose3d(frc::Translation3d(position), frc::Rotation3d()));
    }

    return trajectory;
}

std::vector<frc::Pose3d> Shooter::simulateShot(units::meters_per_second_t launchVel, frc::Pose2d robotPose, units::turn_t hoodAngle,
                                               units::turn_t turretAngle, units::meter_t launchHeight, frc::Translation2d vels) {
    std::vector<frc::Pose3d> trajectory;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> pos;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> vel;

    float vZ = launchVel.to<float>() * units::math::sin(hoodAngle);
    float vX = launchVel.to<float>() * units::math::cos(hoodAngle) * units::math::cos(turretAngle) + vels.X().to<float>();
    float vY = launchVel.to<float>() * units::math::cos(hoodAngle) * units::math::sin(turretAngle) + vels.Y().to<float>();

    Eigen::Vector3d v(vX, vY, vZ);
    Eigen::Vector3d p(robotPose.X().to<float>(), robotPose.Y().to<float>(), launchHeight.to<float>());

    pos.push_back(p);
    vel.push_back(v);

    while (p[2] > 0) {
        auto v_dir = v / v.norm();
        auto drag_mag = 0.5 * RHO * 3.14 * std::pow(R, 2) * CD * std::pow(v.norm(), 2);

        Eigen::Vector3d drag = -(drag_mag * v_dir) / M;
        Eigen::Vector3d aG = Eigen::Vector3d(0, 0, -G);
        Eigen::Vector3d a = drag + aG;

        v = v + a * DT;
        p = p + v * DT;

        vel.push_back(v);
        pos.push_back(p);
    }

    for (auto position : pos) {
        trajectory.push_back(frc::Pose3d(frc::Translation3d(position), frc::Rotation3d()));
    }

    return trajectory;
}

frc2::CommandPtr Shooter::hoodPitSequence() {
    constexpr int NUM_TESTS = 3;
    auto runTest = [=, this](units::degree_t target, int currentTest) {
        return frc2::cmd::Sequence(
            frc2::cmd::RunOnce([=, this] {
                hoodStatus =
                    frc::Alert{fmt::format("Finished {} of {} hood tests", currentTest - 1, NUM_TESTS), frc::Alert::AlertType::kInfo};
                hoodStatus.Set(true);
                manual.hoodAngle = target;
            }),
            frc2::cmd::Wait(2_s), frc2::cmd::RunOnce([=, this] {
                hoodStatus = frc::Alert{fmt::format("Finished {} of {} hood tests", currentTest, NUM_TESTS), frc::Alert::AlertType::kInfo};
                hoodStatus.Set(true);
            }));
    };
    return frc2::cmd::Sequence(frc2::cmd::RunOnce([this] { state.hoodState = HOOD_STATE::MANUAL; }),
                               runTest(Constants::Shooter::getHoodReverseLimit(), 1), runTest(Constants::Shooter::getHoodForwardLimit(), 2),
                               runTest(Constants::Shooter::getHoodReverseLimit(), 3), frc2::cmd::RunOnce([this] { resetState(); }));
}

frc2::CommandPtr Shooter::flywheelPitSequence() {
    constexpr int NUM_TESTS = 2;
    auto runTest = [=, this](units::turns_per_second_t target, units::second_t time, int currentTest) {
        return frc2::cmd::Sequence(
            frc2::cmd::RunOnce([=, this] {
                flywheelStatus =
                    frc::Alert{fmt::format("Finished {} of {} flywheel tests", currentTest - 1, NUM_TESTS), frc::Alert::AlertType::kInfo};
                flywheelStatus.Set(true);
                manual.flywheelSpeed = target;
            }),
            frc2::cmd::Wait(time), frc2::cmd::RunOnce([=, this] {
                flywheelStatus =
                    frc::Alert{fmt::format("Finished {} of {} flywheel tests", currentTest, NUM_TESTS), frc::Alert::AlertType::kInfo};
                flywheelStatus.Set(true);
            }));
    };
    return frc2::cmd::Sequence(frc2::cmd::RunOnce([this] { state.flywheelState = FLYWHEEL_STATE::MANUAL; }), runTest(50_tps, 4_s, 1),
                               runTest(10_tps, 2_s, 2));
}

void Shooter::setManualFlywheelSpeed(units::turns_per_second_t setpoint) {
    state.flywheelState = FLYWHEEL_STATE::MANUAL;
    manual.flywheelSpeed = setpoint;
}

void Shooter::setManualHoodAngle(units::turn_t setpoint) {
    state.hoodState = HOOD_STATE::MANUAL;
    manual.hoodAngle = setpoint;
}

void Shooter::enableAutoShooting() {
    state.flywheelState = FLYWHEEL_STATE::SHOOT;
}

void Shooter::setNextPosition(frc::Pose3d pose) {
    state.nextPosition = pose;
}

void Shooter::setNextVelocity(frc::Translation2d vels) {
    state.nextVelocity = vels;
}

void Shooter::makeHubShoot() {
    state.shootHub = true;
}

void Shooter::makeShuttle() {
    state.shootHub = false;
}

units::meters_per_second_t Shooter::convertTpsToMps(units::turns_per_second_t tps, units::meter_t dist) {
    return units::meters_per_second_t{tps.value() * 2 * std::numbers::pi * FLYWHEEL_RADIUS.value() *
                                      ReadLog("Velocity Loss", state.velocityLoss)};
}

units::turns_per_second_t Shooter::convertMpsToTps(units::meters_per_second_t mps, units::meter_t dist) {
    return units::turns_per_second_t{mps.value() /
                                     (2 * std::numbers::pi * FLYWHEEL_RADIUS.value() * ReadLog("Velocity Loss", state.velocityLoss))};
}

units::turns_per_second_t Shooter::getFlywheelTarget() {
    return state.flywheelTarget;
}
