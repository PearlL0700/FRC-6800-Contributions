#include <frc/system/plant/LinearSystemId.h>
#include "subsystems/Intake.h"
#include "constants/Constants.h"
#include "constants/IntakeConstants.h"
#include "frc/DriverStation.h"
#include "frc/MathUtil.h"
#include "frc/Timer.h"
#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Rotation3d.h"
#include "frc/geometry/Transform3d.h"
#include "frc/geometry/Translation3d.h"
#include "frc2/command/CommandScheduler.h"
#include "units/angular_velocity.h"
#include "valkyrie/controllers/PIDF.h"

#include "ShiftHelpers.h"

#define INTAKE_MIN_TPS 30_tps
#define OUTTAKE_TPS -55_tps

#define INTAKE_MAX_TPS 62_tps
#define INTAKE_FILTER 0.85
#define SPEED_OFFSET 0.5_mps
#define INTAKE_SHOOT_SPEED 10_tps

static constexpr units::volt_t OUTTAKE_POWER = -12_V;

static constexpr units::turns_per_second_t FEEDER_SHOOT_SPEED = 25.0_tps;
static constexpr units::turns_per_second_t HOPPER_SHOOT_SPEED = 12.0_tps;

// 3.28

#define DEPLOYED_POS -0.3605_tr
#define RETRACTED_POS -0.183_tr

#define HOMING_SPEED -1_tps

#define SHIMMY_OUT_POS DEPLOYED_POS
#define SHIMMY_IN_POS RETRACTED_POS
static constexpr units::meters_per_second_t SHIMMY_TELEOP_SPEED_THRESHOLD{.5_mps};
// delta for shimmy = 0.4_tr

// TODO: Shimmy when speed is less than 0.5 mps and canIShoot
// WARN: If can I shoot flickers this flickers

// TODO: Grab Two in auto don't go back to the bump go on the side that we were on
// We can keep the other path but ensure that the end point reaches just before the bump.

#define SHOOTING_SPEED_LIMIT 1.1_mps
#define SHUTTLING_SPEED_LIMIT 2_mps
#define SHOOTING_ACCEL_LIMIT 0.75_mps_sq

static constexpr units::second_t SHIMMY_INTERVAL{0.25_s};
static constexpr units::radian_t ROT_TOLERANCE{3_deg};
static constexpr units::radian_t PIVOT_FLOP_DEG{2_deg};

using namespace valor;

Intake::Intake(frc::TimedRobot& robot, Drivetrain& _drivetrain, Shooter& _shooter, valor::CANdleSensor& _leds)
      : valor::BaseSubsystem("Intake"),
        leftIntakeMotor{valor::PhoenixControllerType::KRAKEN_X60_FOC, CANIDs::INTAKE_LEFT, valor::NeutralMode::Coast,
                        Constants::Feeder::getLeftIntakeInverted(), "rio"},
        rightIntakeMotor{valor::PhoenixControllerType::KRAKEN_X60_FOC, CANIDs::INTAKE_RIGHT, valor::NeutralMode::Coast,
                         !Constants::Feeder::getLeftIntakeInverted(), "rio"},
        leftFeederMotor{valor::PhoenixControllerType::KRAKEN_X60_FOC, CANIDs::FEEDER_LEFT, valor::NeutralMode::Coast,
                        Constants::Feeder::getLeftFeederInverted(), "baseCAN"},
        rightFeederMotor{valor::PhoenixControllerType::KRAKEN_X60_FOC, CANIDs::FEEDER_RIGHT, valor::NeutralMode::Coast,
                         !Constants::Feeder::getLeftFeederInverted(), "baseCAN"},
        leftHopperMotor{valor::PhoenixControllerType::KRAKEN_X44_FOC, CANIDs::HOPPER_LEFT, valor::NeutralMode::Coast,
                        Constants::Feeder::getLeftHopperInverted(), "rio"},
        rightHopperMotor{valor::PhoenixControllerType::KRAKEN_X44_FOC, CANIDs::HOPPER_RIGHT, valor::NeutralMode::Coast,
                         !Constants::Feeder::getLeftHopperInverted(), "rio"},
        pivotMotor{valor::PhoenixControllerType::KRAKEN_X60_FOC, CANIDs::INTAKE_PIVOT, valor::NeutralMode::Brake,
                   Constants::Feeder::getPivotInverted(), "baseCAN"},
        fuelDetector{robot, CANIDs::FEEDER_CANRANGE, "baseCAN", 26_in},

        drivetrain{_drivetrain},
        shooter{_shooter},
        leds{_leds},
        leftIntakeMotorSim{
            frc::LinearSystemId::DCMotorSystem(leftIntakeMotor.motorSpec, 0.0001_kg_sq_m, Constants::Feeder::getIntakeGearRatio()),
            leftIntakeMotor.motorSpec},
        rightIntakeMotorSim{
            frc::LinearSystemId::DCMotorSystem(rightIntakeMotor.motorSpec, 0.0001_kg_sq_m, Constants::Feeder::getIntakeGearRatio()),
            leftIntakeMotor.motorSpec},
        leftFeederMotorSim{
            frc::LinearSystemId::DCMotorSystem(leftFeederMotor.motorSpec, 0.0001_kg_sq_m, Constants::Feeder::getFeederMotorGearRatio()),
            leftFeederMotor.motorSpec},
        rightFeederMotorSim{
            frc::LinearSystemId::DCMotorSystem(rightFeederMotor.motorSpec, 0.0001_kg_sq_m, Constants::Feeder::getFeederMotorGearRatio()),
            rightFeederMotor.motorSpec},
        leftHopperMotorSim{
            frc::LinearSystemId::DCMotorSystem(leftHopperMotor.motorSpec, 0.0001_kg_sq_m, Constants::Feeder::getHopperMotorGearRatio()),
            leftHopperMotor.motorSpec},
        rightHopperMotorSim{
            frc::LinearSystemId::DCMotorSystem(rightHopperMotor.motorSpec, 0.0001_kg_sq_m, Constants::Feeder::getHopperMotorGearRatio()),
            rightHopperMotor.motorSpec},
        pivotMotorSim{frc::LinearSystemId::DCMotorSystem(pivotMotor.motorSpec, 0.0001_kg_sq_m,
                                                         Constants::Feeder::pivotMotortoSensor() * Constants::Feeder::pivotSensortoMech()),
                      pivotMotor.motorSpec},
        shootingSequence{shootSequence()} {
    init();
}

void Intake::resetState() {
    state.pivotState = PIVOT_STATE::RETRACTED;
    state.intakeState = INTAKE_STATE::OFF;
    state.forcedShoot = false;
    state.intakeSpeed = 0_tps;
}

void Intake::updateFilteredRobotSpeed() {
    const units::meters_per_second_t raw = drivetrain.getRobotSpeeds();
    filteredRobotSpeed = filteredRobotSpeed * INTAKE_FILTER + raw * (1.0 - INTAKE_FILTER);
}

units::turns_per_second_t Intake::getIntakeTps() const {
    double t = (filteredRobotSpeed / (drivetrain.getMaxDriveSpeed() - SPEED_OFFSET)).value();
    t = std::clamp(t, 0.0, 1.0);

    units::turns_per_second_t cmd = INTAKE_MIN_TPS + (INTAKE_MAX_TPS - INTAKE_MIN_TPS) * t;

    return cmd;
}

void Intake::init() {
    for (auto motor : {&leftIntakeMotor, &rightIntakeMotor}) {
        motor->setGearRatios(1, Constants::Feeder::getIntakeGearRatio());
        valor::PIDF intakePID = Constants::Feeder::getIntakePID();
        motor->setPIDF(intakePID);
        valor::util::PhoenixSignalManager::GetInstance().AddSignals(motor->GetNetwork(), motor->GetSupplyCurrent(false));
        motor->setCurrentLimits(50_A, 45_A, 35_A, 1.75_s);
        motor->applyConfig();
    }

    pivotMotor.setGearRatios(Constants::Feeder::pivotMotortoSensor(), Constants::Feeder::pivotSensortoMech());
    pivotMotor.setForwardLimit(Constants::Feeder::getPivotForwardLimit());
    pivotMotor.setReverseLimit(Constants::Feeder::getPivotReverseLimit());
    pivotMotor.setCurrentLimits(60_A, 60_A, 40_A, 1_s);
    valor::PIDF pivotPID = Constants::Feeder::getPivotPID();
    pivotMotor.setPIDF(pivotPID);
    pivotMotor.applyConfig();

    for (auto& feederMotor : {&leftFeederMotor, &rightFeederMotor}) {
        feederMotor->SetPeriodicLevel(valor::LogLevel::Important);
        feederMotor->setGearRatios(1.0, Constants::Feeder::getFeederMotorGearRatio());

        valor::PIDF feederPID = Constants::Feeder::getFeederPID();
        feederPID.maxVelocity = feederMotor->getMaxMechSpeed();
        feederPID.maxAcceleration = feederMotor->getMaxMechSpeed() / 1_s;
        feederMotor->setCurrentLimits(50_A, 40_A, 40_A, 1_s);
        feederMotor->setPIDF(feederPID);
        feederMotor->applyConfig();
    }

    for (auto& hopperMotor : {&leftHopperMotor, &rightHopperMotor}) {
        hopperMotor->setGearRatios(1.0, Constants::Feeder::getHopperMotorGearRatio());
        valor::PIDF hopperPID = Constants::Feeder::getHopperPID();
        hopperPID.maxVelocity = hopperMotor->getMaxMechSpeed();
        hopperPID.maxAcceleration = hopperMotor->getMaxMechSpeed() / 1_s;
        hopperMotor->setCurrentLimits(35_A, 40_A, 65_A, 1_s);
        hopperMotor->setPIDF(hopperPID);
        hopperMotor->applyConfig();
    }

    LogChild("Left Intake Motor", &leftIntakeMotor);
    LogChild("Right Intake Motor", &rightIntakeMotor);
    LogChild("Left Hopper Motor", &leftHopperMotor);
    LogChild("Right Hopper Motor", &rightHopperMotor);
    LogChild("Left Feeder Motor", &leftFeederMotor);
    LogChild("Right Feeder Motor", &rightFeederMotor);
    LogChild("Pivot Motor", &pivotMotor);

    WriteLog("Pivot Flop Deployment", true);
    WriteLog("All Outtake", false);
    WriteLog("Enable Rotation Align", true);

    feederIntakeSpeeds = {{FEEDER_SHOOT_SPEED, FEEDER_SHOOT_SPEED}};
    hopperIntakeSpeeds = {{HOPPER_SHOOT_SPEED, HOPPER_SHOOT_SPEED}};

    feederMotors = {{&leftFeederMotor, &rightFeederMotor}};
    hopperMotors = {{&leftHopperMotor, &rightHopperMotor}};

    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);

    resetState();
}

void Intake::SimulationPeriodic() {
    auto step = [&](auto& motor, frc::sim::DCMotorSim& sim) {
        sim.SetInputVoltage(motor.calculateAppliedVoltage());
        sim.Update(frc::TimedRobot::kDefaultPeriod);
        motor.setSimState(sim);
    };

    step(leftIntakeMotor, leftIntakeMotorSim);
    step(leftFeederMotor, leftFeederMotorSim);
    step(rightFeederMotor, rightFeederMotorSim);
    step(leftHopperMotor, leftHopperMotorSim);
    step(rightHopperMotor, rightHopperMotorSim);
    step(pivotMotor, pivotMotorSim);
}

void Intake::assessInputs() {
    if (driverGamepad->rightTriggerActive() || driverGamepad->GetRightBumperButton() || driverGamepad->leftTriggerActive() ||
        driverGamepad->GetLeftBumperButton()) {
        shooter.state.flywheelState = Shooter::FLYWHEEL_STATE::SHOOT;
        shooter.state.hoodState = Shooter::HOOD_STATE::TRACKING;
    } else {
        shooter.state.flywheelState = Shooter::FLYWHEEL_STATE::DISABLE;
        shooter.state.hoodState = Shooter::HOOD_STATE::HOME;
        state.dumperState = DUMPER_STATE::OFF;
    }

    if (driverGamepad->rightTriggerActive() || driverGamepad->GetRightBumperButton()) {
        state.dumperState = DUMPER_STATE::SHOOTING;
    }
    state.forcedShoot = driverGamepad->GetRightBumperButton();

    if (operatorGamepad->rightTriggerActive()) {
        state.pivotState = PIVOT_STATE::OFF;
    } else if (driverGamepad->DPadDown()) {
        state.pivotState = PIVOT_STATE::RETRACTED;
    } else {
        state.pivotState = PIVOT_STATE::DEPLOYED;
    }

    if (driverGamepad->GetXButton()) {
        state.intakeState = INTAKE_STATE::OUTTAKING;
        state.feederState = FEEDER_STATE::OUTTAKING;
        state.hopperState = HOPPER_STATE::OUTTAKING;
    } else if (operatorGamepad->GetBButton()) {
        state.intakeState = INTAKE_STATE::OFF;
    } else if (shooter.pitMode) {
        state.intakeState = INTAKE_STATE::SHOOTING;
    }
}

void Intake::analyzeDashboard() {
    frc::Pose3d pivotPosition = frc::Pose3d{} + Constants::Feeder::pivotPosition();
    WriteLog("Intake Robot Relative",
             pivotPosition.RotateAround(pivotPosition.Translation(), frc::Rotation3d{0_deg, pivotMotor.getPosition(), 0_deg}));

    WriteLog("Intake State", state.intakeState);
    WriteLog("Pivot State", state.pivotState);
    WriteLog("Feeder State", state.feederState);
    WriteLog("Hopper State", state.hopperState);
    WriteLog("Dumper State", state.dumperState);

    leftIntakeMotor.WriteLog("Supply Current", leftIntakeMotor.GetSupplyCurrent(false).GetValue());
    rightIntakeMotor.WriteLog("Supply Current", rightIntakeMotor.GetSupplyCurrent(false).GetValue());

    state.flopDeploymentActivation = ReadLog("Pivot Flop Deployment", state.flopDeploymentActivation);

    if (state.dumperState == DUMPER_STATE::SHOOTING) {
        if (state.forcedShoot) {
            state.feederState = FEEDER_STATE::SHOOTING;
            state.hopperState = HOPPER_STATE::SHOOTING;
            state.intakeState = INTAKE_STATE::SHOOTING;
            // frc2::CommandScheduler::GetInstance().Schedule(shimmyIntake());
        } else if (canIShoot()) {
            frc2::CommandScheduler::GetInstance().Schedule(shootingSequence);
        }
    } else {
        if (shootingSequence.IsScheduled()) {
            shootingSequence.Cancel();
        }
        state.feederState = FEEDER_STATE::DISABLED;
        state.hopperState = HOPPER_STATE::DISABLED;
        state.intakeState = INTAKE_STATE::INTAKING;
    }

    if (state.intakeState == INTAKE_STATE::OUTTAKING) {
    } else if (state.intakeState == INTAKE_STATE::INTAKING) {
        updateFilteredRobotSpeed();
        if (frc::DriverStation::IsTeleop())
            state.intakeSpeed = INTAKE_MAX_TPS;
        // state.intakeSpeed = getIntakeTps();
    } else if (state.intakeState == INTAKE_STATE::OFF) {
        state.intakeSpeed = 0_tps;
    } else if (state.intakeState == INTAKE_STATE::SHOOTING) {
        state.intakeSpeed = INTAKE_SHOOT_SPEED;
    }

    units::ampere_t averagedCurrent = intakeCurrentSensor.Calculate(leftIntakeMotor.getCurrent());

    if (averagedCurrent >= 50_A && leftIntakeMotor.getSpeed() < 1_tps) {
        WriteLog("Intake Jam", true);
        driverGamepad->setRumble(true, 0.1);
    } else {
        WriteLog("Intake Jam", false);
        driverGamepad->setRumble(false, 0.1);
    }

    setLEDs();
    // Run CANCoder magnet health check at ~2Hz (driven by drivetrain counter)
    if (drivetrain.diagLoopCounter == 0)
        // leds.setLED(4, valor::CANdleSensor::cancoderMagnetHealthGetter(pivotMotor.getCANCoder()));
        leds.setLED(5, valor::CANdleSensor::canrangeHealthGetter(fuelDetector));
}

void Intake::assignOutputs() {
    if (state.intakeState == INTAKE_STATE::OUTTAKING) {
        leftIntakeMotor.setPower(OUTTAKE_POWER);
        rightIntakeMotor.setPower(OUTTAKE_POWER);
    } else if (state.intakeSpeed != 0_tps) {
        leftIntakeMotor.setSpeed(state.intakeSpeed);
        rightIntakeMotor.setSpeed(state.intakeSpeed);
    } else {
        leftIntakeMotor.setPower(0_V);
        rightIntakeMotor.setPower(0_V);
    }

    if (state.feederState == FEEDER_STATE::OUTTAKING) {
        setFeederPower(OUTTAKE_POWER);
    } else if (state.feederState == FEEDER_STATE::SHOOTING) {
        setFeederSpeeds(feederIntakeSpeeds);
    } else {
        setFeederPower(0_V);
    }

    if (state.hopperState == HOPPER_STATE::OUTTAKING) {
        setHopperPower(OUTTAKE_POWER);
    } else if (state.hopperState == HOPPER_STATE::SHOOTING) {
        setHopperSpeeds(hopperIntakeSpeeds);
    } else {
        setHopperPower(0_V);
    }

    if (state.pivotState == PIVOT_STATE::SHIMMY_IN) {
        pivotMotor.setPosition(SHIMMY_IN_POS);
    } else if (state.pivotState == PIVOT_STATE::SHIMMY_OUT) {
        pivotMotor.setPosition(SHIMMY_OUT_POS);
    } else if (state.pivotState == PIVOT_STATE::DEPLOYED) {
        if (state.flopDeploymentActivation && frc::IsNear(DEPLOYED_POS, pivotMotor.getPosition(), units::turn_t{PIVOT_FLOP_DEG})) {
            pivotMotor.setPower(0_V);
        } else {
            pivotMotor.setPosition(DEPLOYED_POS);
        }
    } else if (state.pivotState == PIVOT_STATE::OFF) {
        pivotMotor.setPower(0_V);
    } else {
        pivotMotor.setPosition(RETRACTED_POS);
    }
}

frc2::CommandPtr Intake::shootSequence() {
    return frc2::SequentialCommandGroup(frc2::InstantCommand([this]() { state.intakeState = INTAKE_STATE::OFF; }), frc2::WaitCommand(0.1_s),
                                        frc2::InstantCommand([this]() { state.feederState = FEEDER_STATE::SHOOTING; }),
                                        frc2::WaitCommand(0.1_s),
                                        frc2::InstantCommand([this]() { state.hopperState = HOPPER_STATE::SHOOTING; }),
                                        frc2::InstantCommand([this]() { state.intakeState = INTAKE_STATE::SHOOTING; }))
        .ToPtr();
}

frc2::CommandPtr Intake::pitSequence() {
    constexpr int NUM_TESTS = 5;
    auto runTest = [=, this](int currentTest, units::second_t time, std::function<void()> runner) {
        return frc2::cmd::Sequence(frc2::cmd::RunOnce([=, this] {
                                       intakeStatus = frc::Alert{fmt::format("Finished {} of {} intake tests", currentTest - 1, NUM_TESTS),
                                                                 frc::Alert::AlertType::kInfo};
                                       intakeStatus.Set(true);
                                       runner();
                                   }),
                                   frc2::cmd::Wait(time), frc2::cmd::RunOnce([=, this] {
                                       intakeStatus = frc::Alert{fmt::format("Finished {} of {} intake tests", currentTest, NUM_TESTS),
                                                                 frc::Alert::AlertType::kInfo};
                                       intakeStatus.Set(true);
                                   }));
    };
    return frc2::cmd::Sequence(frc2::cmd::RunOnce([this] { state.intakeState = INTAKE_STATE::MANUAL; }),
                               runTest(1, 2_s,
                                       [this] {
                                           state.intakeSpeed = INTAKE_MIN_TPS;
                                           state.pivotState = PIVOT_STATE::RETRACTED;
                                       }),
                               runTest(2, 2_s,
                                       [this] {
                                           state.intakeSpeed = INTAKE_MAX_TPS;
                                           state.pivotState = PIVOT_STATE::DEPLOYED;
                                       }),
                               runTest(3, 2_s,
                                       [this] {
                                           state.intakeSpeed = INTAKE_MIN_TPS;
                                           state.pivotState = PIVOT_STATE::RETRACTED;
                                       }),
                               runTest(4, 2_s,
                                       [this] {
                                           state.intakeSpeed = INTAKE_MAX_TPS;
                                           state.pivotState = PIVOT_STATE::DEPLOYED;
                                       }),
                               runTest(5, 6_s, [this] {
                                   state.intakeSpeed = INTAKE_MIN_TPS;
                                   shoot();
                               }));
}

frc2::CommandPtr Intake::shimmyIntake() {
    return frc2::SequentialCommandGroup(
               frc2::InstantCommand([this]() { state.pivotState = PIVOT_STATE::SHIMMY_IN; }), frc2::WaitCommand(SHIMMY_INTERVAL),
               frc2::InstantCommand([this]() { state.pivotState = PIVOT_STATE::SHIMMY_OUT; }), frc2::WaitCommand(SHIMMY_INTERVAL))
        .ToPtr();
}

bool Intake::canIShoot() {
    if (frc::DriverStation::IsAutonomous()) {
        return state.forcedShoot;
    }
    bool shiftOk = ShiftHelpers::canShoot() || ShiftHelpers::isGameDataUnknown();

    shootingRequirements[0] = shooter.isSystemReady();
    shootingRequirements[1] = shiftOk;
    shootingRequirements[2] = inAllianceZone();
    shootingRequirements[3] = drivetrain.isSpeedBelowThreshold(SHOOTING_SPEED_LIMIT);
    shootingRequirements[4] = drivetrain.isAccelBelowThreshold(SHOOTING_ACCEL_LIMIT);
    shootingRequirements[5] =
        ReadLog("Enable Rotation Align", true)
            ? frc::IsNear(drivetrain.state.rotationTarget, drivetrain.getCalculatedPose().Rotation().Radians(), ROT_TOLERANCE)
            : true;
    WriteLog("Can I Shoot", shootingRequirements.to_string());

    return shootingRequirements.all();
}

bool Intake::canIShuttle() {
    if (frc::DriverStation::IsAutonomous()) {
        return state.forcedShoot;
    }

    shuttlingRequirements[0] = shooter.isSystemReady();
    shuttlingRequirements[1] = !inAllianceZone();
    shuttlingRequirements[2] = drivetrain.isSpeedBelowThreshold(SHUTTLING_SPEED_LIMIT);
    WriteLog("Can I Shuttle", shuttlingRequirements.to_string());

    return shuttlingRequirements.all();
}

bool Intake::inAllianceZone() {
    return shooter.inAllianceZone(shooter.getWorldShooterPose().ToPose2d());
}

void Intake::setLEDs() {
    if (!ShiftHelpers::canShoot()) {
        leds.setColor(1, valor::CANdleSensor::ORANGE);
    } else if (shooter.isSystemReady()) {
        leds.setColor(1, valor::CANdleSensor::GREEN);
    } else {
        leds.setColor(1, valor::CANdleSensor::RED);
    }
}

void Intake::setFeederSpeeds(std::array<units::angular_velocity::turns_per_second_t, 2UL> feedSpeeds) {
    for (size_t i = 0; i < feederMotors.size(); i++) {
        feederMotors[i]->setSpeed(feedSpeeds[i]);
    }
}

void Intake::setFeederPower(units::volt_t power) {
    for (auto* m : feederMotors) {
        m->setPower(power);
    }
}

void Intake::setHopperSpeeds(std::array<units::angular_velocity::turns_per_second_t, 2UL> hopperSpeeds) {
    for (size_t i = 0; i < hopperMotors.size(); i++) {
        hopperMotors[i]->setSpeed(hopperSpeeds[i]);
    }
}

void Intake::setHopperPower(units::volt_t power) {
    for (auto* m : hopperMotors) {
        m->setPower(power);
    }
}

void Intake::deployIntake() {
    state.pivotState = PIVOT_STATE::DEPLOYED;
}

void Intake::retractIntake() {
    state.pivotState = PIVOT_STATE::SHIMMY_IN;
}

void Intake::enableIntake() {
    state.intakeState = INTAKE_STATE::INTAKING;
}

void Intake::disableIntake() {
    state.intakeState = INTAKE_STATE::OFF;
}

void Intake::shoot() {
    state.forcedShoot = true;
}

void Intake::stopShoot() {
    state.forcedShoot = false;
}

void Intake::setIntakeSpeed(units::turns_per_second_t speed) {
    state.intakeSpeed = speed;
}
