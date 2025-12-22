# 2025 FRC Team 6800 Robot – Climber Subsystem

Software for the climber mechanism of Team Valor 6800’s 2025 FIRST Robotics Competition (FRC) robot, designed to safely and reliably lift the robot onto the endgame structure under high mechanical load and time pressure.

## Project Overview & Scope

This repository contains the climber subsystem code that I independently designed, implemented, wired, and tested for Team Valor 6800’s 2025 competition robot. The robot competed in multiple Texas District events and advanced to the FIRST Championship, where climb reliability and fault tolerance were critical to match success.

Due to team intellectual property and collaboration policies, this repository does **not** include the full robot project and will not build or run independently. Video demonstrations and photos showing the climber operating on the physical robot are provided in my MIT Maker Portfolio.

## Subsystem Objective

The climber subsystem is responsible for lifting the robot onto the endgame structure (a cage) through controlled, software-mediated motion. The software is designed to:

- Coordinate motors and sensors safely under high load
- Prevent mechanical overextension and back-driving
- Remain responsive to driver and operator input while enforcing safety constraints
- Recover safely from partial or interrupted climb attempts
- Achieve positional accuracy within ±1° of the commanded climber position
- Complete the full climb motion in under 5 seconds under nominal match conditions

## Control Architecture

### State-Based Climb Control

The climber is implemented as a **supervisory, event-driven finite state machine (FSM)** that supervises actuator behavior based on human intent and sensor feedback.

Rather than executing a fully autonomous climb sequence, the system uses **human-in-the-loop control**: Driver and Operator inputs request high-level states, while the software enforces safety constraints and motion limits.

### Gating & Safety Logic

A software lockout mechanism prevents accidental or premature climber activation. State transitions are only allowed after an explicit Driver/Operator enable, and manual control remains gated by the same safety logic.

This feature was introduced after the first district competition, when I identified a high-risk failure point under match stress: an accidental climber deployment. Because the climber uses a one-way latching mechanism that allows only a single deploy–retract cycle, unintended activation would permanently disable the subsystem for the remainder of the match. The lockout mitigates this risk.

```cpp
if(driverGamepad->GetStartButton() || operatorGamepad->GetStartButton()){
        state.lockOut = true;
    }
```

### Sensor-Based Overrides

An absolute encoder (CANCoder) provides global positional feedback. Once the climber passes a defined lockout threshold or the robot has successfully completed the climb, motor output is forcibly disabled and the subsystem enters a safe holding state. This prevents mechanical overextension, back-driving, and post-climb motion.

### Separate PID slots

During testing, I saw that the PID gains used for pivoting the climber were not aggressive enough to reliably retract the mechanism under full robot load (when it’s lifting the robot off the ground). To address this, I implemented separate PID slots with different gain values, allowing the controller to apply more aggressive control during retraction while maintaining stability during unloaded motion.
 
Here is an excerpt of code, located in a separate configuration file, that defines these distinct PID slots and enables dynamic selection based on the current climber state.

```cpp
void setPIDF(valor::PIDF _pidf, int slot = 0, bool saveImmediately = false) override {
        pidf = _pidf;

        // Motion magic configuration
        config.MotionMagic.MotionMagicCruiseVelocity = pidf.maxVelocity;
        config.MotionMagic.MotionMagicAcceleration = pidf.maxAcceleration;
        config.MotionMagic.MotionMagicJerk = pidf.maxJerk;

        if (slot == 1) setPIDFSlot(config.Slot1, saveImmediately);
        else if (slot == 2) setPIDFSlot(config.Slot2, saveImmediately);
        else setPIDFSlot(config.Slot0, saveImmediately); // Default case

        if (saveImmediately) getMotor()->GetConfigurator().Apply(config.MotionMagic);
    }
```
## Acknowledgments

- Team Valor 6800 
- CTRE Phoenix Library
- WPILib open-source community
