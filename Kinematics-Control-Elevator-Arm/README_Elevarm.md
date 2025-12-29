# Kinematics Control for Elevator + Arm – FRC Team Valor 6800 (2023)

Software for the elevator and rotating arm of Team Valor 6800’s 2023 FIRST Robotics Competition (FRC) robot, designed to intake, control, and score game pieces reliably.

## Project Overview & Scope

This repository contains the Elevator and Arm subsystem code that I designed, implemented, and tested **with another student** for Team Valor 6800’s 2023 competition robot. The robot competed in multiple Texas District events and advanced to the FIRST Championship, where scoring reliability and fault tolerance were critical to match success.

Due to team intellectual property and collaboration policies, this repository does **not** include the full robot project and will not build or run independently. Video demonstrations and photos showing the subsystem operating on the physical robot are provided in my MIT Maker Portfolio.

Below is the code I primarily designed and implemented:
```cpp
Elevarm::Positions Elevarm::inverseKinematics(frc::Pose2d pose, ElevarmSolutions solution, Direction dir) 
{
    double phi = 0.0;
    double theta = 0.0;
    double height = 0.0;
    double direction = (dir == Direction::FRONT) ? 1.0 : -1.0;

    // Arms solution
    if (solution == ElevarmSolutions::ELEVARM_ARMS) {
        phi = std::acos(std::fabs((pose.X().to<double>() + X_BUMPER_WIDTH + X_HALF_WIDTH - X_CARRIAGE_OFFSET) / X_ARM_LENGTH));
        theta = phi + (M_PI / 2.0);
        height = pose.Y().to<double>() - (X_ARM_LENGTH * std::sin(phi));
        theta *= direction;
    // Legs Solution
    } else {
        theta = std::asin((pose.X().to<double>() + X_BUMPER_WIDTH + X_HALF_WIDTH - X_CARRIAGE_OFFSET) / X_ARM_LENGTH);
        height = pose.Y().to<double>() + (X_ARM_LENGTH * std::cos(theta));
    }
    height -= (Z_CARRIAGE_FLOOR_OFFSET + Z_CARRIAGE_JOINT_OFFSET);
    height += futureState.carriageOffset / 100.0; //convert fron cm on dashboard to m in logic


    return Positions(height,theta * 180.0 / M_PI, pose.Rotation().Degrees().to<double>());
}

frc::Pose2d Elevarm::forwardKinematics(Elevarm::Positions positions) 
{
    double x, z = 0;
    double theta = positions.theta * M_PI / 180.0;
    double w = wristMotor.getPosition();

    // Forward
    if (theta > 0) {
        // Arms
        if (theta > (M_PI / 2.0)) {
            double phi = theta - (M_PI / 2.0);
            z = Z_CARRIAGE_FLOOR_OFFSET + Z_CARRIAGE_JOINT_OFFSET + positions.h + X_ARM_LENGTH * sin(phi);
            x = X_ARM_LENGTH * cos(phi) + X_CARRIAGE_OFFSET - X_BUMPER_WIDTH - X_HALF_WIDTH;
        // Legs
        } else {
            z = Z_CARRIAGE_FLOOR_OFFSET + Z_CARRIAGE_JOINT_OFFSET + positions.h - X_ARM_LENGTH * cos(theta);
            x = X_ARM_LENGTH * sin(theta) + X_CARRIAGE_OFFSET - X_BUMPER_WIDTH - X_HALF_WIDTH;
        }
    // Reverse
    } else {
        // Arms
        if (theta < -(M_PI / 2.0)) {
            double phi = std::fabs(theta + (M_PI / 2.0));
            z = Z_CARRIAGE_FLOOR_OFFSET + Z_CARRIAGE_JOINT_OFFSET + positions.h + X_ARM_LENGTH * sin(phi);
            x = -X_ARM_LENGTH * cos(phi) + X_CARRIAGE_OFFSET - X_BUMPER_WIDTH - X_HALF_WIDTH;
        // Legs
        } else {
            z = Z_CARRIAGE_FLOOR_OFFSET + Z_CARRIAGE_JOINT_OFFSET + positions.h - X_ARM_LENGTH * cos(std::fabs(theta));
            x = -(X_ARM_LENGTH * sin(std::fabs(theta))) + X_CARRIAGE_OFFSET - X_BUMPER_WIDTH - X_HALF_WIDTH;
        }
    }
    return frc::Pose2d((units::length::meter_t)x,(units::length::meter_t)z,(units::angle::degree_t)w);
}

```

## Subsystem Objective

The elevarm subsystem is responsible for intaking and scoring two distinct types of game pieces: a **cone** and an **inflatable cube**, across multiple heights and field locations.

**My Design Constraints:** 
- Reliably handle two geometrically different game pieces with a single mechanism  
- Coordinate elevator and arm motion without collisions or unsafe configurations  
- Prevent elevator or arm motion that exceeds the robot height limit  
- Maintain continuous awareness of the elevator–arm configuration at all times  
- Maintain precise positioning across a wide range of angles and heights  
- Remain responsive to Driver and Operator input under match time pressure  

## Control Architecture

The elevator + arm subsystem is controlled using a **kinematics-based architecture**, treating the elevator and arm as a single coordinated system rather than two independent actuators. Instead of commanding raw motor positions, the software works in **task space** (end-effector pose) and converts those requests into safe, valid joint commands.

### Forward and Inverse Kinematics

I implemented inverse kinematics and forward kinematics to continuously track the mechanism’s real-world configuration:

- **inverse kinematics (`inverseKinematics`)** converts a desired pose (x, z, wrist angle) into elevator height and arm rotation
  - Supports multiple geometric solutions (arms vs. legs) and front/back operation  
  - Takes into account bumpers and carriage height for field-accurate positioning  
  - Handles direction and solution selection to avoid unsafe arm configurations and ensure predictable motion  

- **Forward kinematics (`forwardKinematics`)** calculates the arm’s real-world position from the current elevator height and arm angle
  - Verifies where the mechanism is in space at all times  
  - Used for validation, visualization, and safety checks (height limits, collision avoidance)  

### Safety and Coordination

By combining forward and inverse kinematics, the system maintains **closed-loop awareness** of the elevator–arm configuration. This prevents illegal states (like exceeding height limits or unsafe angles) and lets the elevator and arm move in coordinated trajectories instead of independently.  

This approach improved reliability and made debugging easier compared to directly commanding motor positions, while also providing a strong foundation for safe, state-based control and future iterations.

## Reflection

This project taught me the value of modeling physical systems mathematically instead of relying on tuning or fixed presets. As we added more functionality, controlling the elevator and arm independently led to too many edge cases, which pushed me to think about the subsystem as a single geometric system.

Implementing inverse kinematics shifted my focus to where the arm should be in space (angle, height etc) rather than how each motor should move, reducing manual tuning and making behavior more predictable. Adding forward kinematics provided continuous feedback on the mechanism’s real position, which made debugging easier and helped catch issues earlier.

Looking back, I would have formalized constraints earlier instead of layering special cases. While our approach worked given the time constraints we had, a cleaner constraint-based model would have made the system easier to maintain and scale. I also would have replaced magic numbers in the code with named variables to make it more readable, so I wouldn’t have to wonder why an offset or conversion factor was added, and to reduce the risk of errors when updating values. Overall, this project changed how I think about robotics software: strong abstractions and physical models directly improve reliability.

## Acknowledgments

- Team Valor 6800 
- CTRE Phoenix Library
- WPILib





