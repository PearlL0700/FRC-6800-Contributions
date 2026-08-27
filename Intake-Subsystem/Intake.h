#pragma once

#include <frc/simulation/DCMotorSim.h>
#include "ShiftHelpers.h"
#include "frc/filter/LinearFilter.h"
#include "units/current.h"
#include "valkyrie/BaseSubsystem.h"
#include "valkyrie/controllers/PhoenixController.h"
#include "constants/IntakeConstants.h"
#include "Drivetrain.h"
#include "Shooter.h"
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>

class Intake : public valor::BaseSubsystem {
   public:
    Intake(frc::TimedRobot& robot, Drivetrain& drivetrain, Shooter& shooter, valor::CANdleSensor& leds);

    void init();
    void resetState();
    void assessInputs() override;
    void analyzeDashboard() override;
    void assignOutputs() override;
    void SimulationPeriodic() override;

    frc2::CommandPtr shootSequence();
    frc2::CommandPtr shimmyIntake();

    void setFeederSpeeds(std::array<units::angular_velocity::turns_per_second_t, 2UL> feedSpeeds);
    void setHopperSpeeds(std::array<units::angular_velocity::turns_per_second_t, 2UL> setHopperSpeeds);
    void setFeederPower(units::volt_t power);
    void setHopperPower(units::volt_t power);

    bool canIShoot();
    bool canIShuttle();
    bool crossedStartLine();
    bool inAllianceZone();
    void setLEDs();

    enum class PIVOT_STATE { RETRACTED, DEPLOYED, SHIMMY_IN, SHIMMY_OUT, OFF };
    enum class INTAKE_STATE { OFF, MANUAL, OUTTAKING, INTAKING, SHOOTING };
    enum class FEEDER_STATE { DISABLED, OUTTAKING, SHOOTING };
    enum class HOPPER_STATE { DISABLED, OUTTAKING, SHOOTING };
    enum class DUMPER_STATE { OFF, PRIMING, SHOOTING };

    struct State {
        PIVOT_STATE pivotState;
        INTAKE_STATE intakeState;
        FEEDER_STATE feederState;
        HOPPER_STATE hopperState;
        DUMPER_STATE dumperState;
        bool forcedShoot;
        units::turns_per_second_t intakeSpeed;
        bool flopDeploymentActivation;
    } state;

    // these functions are used by auto to interact with the class
    void deployIntake();
    void retractIntake();
    void enableIntake();
    void disableIntake();

    void shoot();
    void stopShoot();

    void setIntakeSpeed(units::turns_per_second_t speed);

    frc2::CommandPtr pitSequence();

    ShiftHelpers::GameData gameData{ShiftHelpers::GameData::kUnknown};

   private:
    valor::PhoenixController<ctre::phoenix6::controls::VelocityTorqueCurrentFOC> leftIntakeMotor;
    valor::PhoenixController<ctre::phoenix6::controls::VelocityTorqueCurrentFOC> rightIntakeMotor;
    valor::PhoenixController<ctre::phoenix6::controls::VelocityTorqueCurrentFOC> leftFeederMotor;
    valor::PhoenixController<ctre::phoenix6::controls::VelocityTorqueCurrentFOC> rightFeederMotor;
    valor::PhoenixController<ctre::phoenix6::controls::VelocityTorqueCurrentFOC> leftHopperMotor;
    valor::PhoenixController<ctre::phoenix6::controls::VelocityTorqueCurrentFOC> rightHopperMotor;
    valor::PhoenixController<> pivotMotor;
    valor::CANrangeSensor fuelDetector;

    Drivetrain& drivetrain;
    Shooter& shooter;
    valor::CANdleSensor& leds;

    frc::sim::DCMotorSim leftIntakeMotorSim;
    frc::sim::DCMotorSim rightIntakeMotorSim;
    frc::sim::DCMotorSim leftFeederMotorSim;
    frc::sim::DCMotorSim rightFeederMotorSim;
    frc::sim::DCMotorSim leftHopperMotorSim;
    frc::sim::DCMotorSim rightHopperMotorSim;
    frc::sim::DCMotorSim pivotMotorSim;

    frc2::CommandPtr shootingSequence;

    units::meters_per_second_t filteredRobotSpeed{0_mps};

    units::turns_per_second_t getIntakeTps() const;
    void updateFilteredRobotSpeed();
    std::array<units::turns_per_second_t, 2> feederIntakeSpeeds;
    std::array<units::turns_per_second_t, 2> hopperIntakeSpeeds;

    std::array<valor::PhoenixController<ctre::phoenix6::controls::VelocityTorqueCurrentFOC>*, 2> feederMotors;
    std::array<valor::PhoenixController<ctre::phoenix6::controls::VelocityTorqueCurrentFOC>*, 2> hopperMotors;

    std::bitset<6> shootingRequirements;
    std::bitset<3> shuttlingRequirements;

    frc::LinearFilter<units::ampere_t> intakeCurrentSensor = frc::LinearFilter<units::ampere_t>::MovingAverage(7);

    frc::Alert intakeStatus{"", frc::Alert::AlertType::kInfo};
};
