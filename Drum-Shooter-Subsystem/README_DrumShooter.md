# Drum Shooter Subsystem – FRC Team Valor 6800 (2026)

Software for the shooting mechanism of Team Valor 6800’s 2026 FIRST Robotics Competition (FRC) robot, designed to automatically configure and control the shooter for reliable scoring across varying distances.

## Project Overview & Scope

This repository contains the shooter subsystem code that I designed, implemented, and tested **with other students** for Team Valor 6800’s 2026 competition robot. The robot competed in multiple Texas District events and advanced to the FIRST Championship, where consistent shot placement and rapid scoring cycles were critical to match success.

Due to team intellectual property and collaboration policies, this repository does **not** include the full robot project and will not build or run independently. Video demonstrations and photos showing the subsystem operating on the physical robot are provided in my portfolio.

## Subsystem Objective

The shooter subsystem is responsible for launching multiple **dense foam ball** into a scoring hub from a range of distances while accounting for the robot’s position on the field.

**My Design Constraints:**  
- Reliably score multiple game pieces at once across varying distances
- Automatically determine appropriate flywheel speed and hood angle based on distance to the target
- Maintain consistent shot parameters as the robot moves around the field
- Verify that the flywheel, hood, and chassis has reached its required operating conditions before allowing the robot to shoot
- Support both close-range scoring and longer-distance shuttle shots
- Remain responsive to Driver and Operator input under match time pressure
- Account for robot motion when calculating projectile behavior

## Control Architecture

### State-Based Control

The shooter subsystem is implemented as a **supervisory, event-driven state machine** that independently controls the flywheel and hood based on Driver/Operator intent and the robot’s position relative to the selected target.

The system uses separate states for the flywheel and hood, including **disabled, tracking, manual, home, and pit-mode behaviors**. During normal operation, the shooter continuously calculates the required flywheel velocity and hood position from the robot’s distance to the selected target and commands both mechanisms toward those setpoints.

### Distance-Based Shot Interpolation

Rather than requiring the Driver to manually determine shooter settings for every shot, I implemented **distance-based interpolation** using experimentally determined shot parameters.

The subsystem maintains separate lookup tables for **hub shots and shuttle shots**, mapping distance to both flywheel velocity and hood angle. The system selects the appropriate interpolation table based on the robot’s location on the field.

**Implementation Details:**  
- **Distance measurement:** Calculates the distance between the shooter and the selected scoring target using the robot’s estimated field position
- **Shot interpolation:** Determines flywheel velocity and hood angle from experimentally tuned distance-based lookup tables
- **Hub and shuttle modes:** Uses separate shot profiles for close-range hub scoring and longer-range shuttle shots
- **Continuous adjustment:** Recalculates the required shot parameters as the robot moves relative to the target
- **Motor control:** Closed-loop velocity control maintains the flywheel at its calculated target speed while position control adjusts the hood angle

**Impact:**  
- Reduced Driver workload when selecting shot parameters
- Enabled consistent shooting across a wide range of distances
- Allowed the shooter to automatically adapt as the robot moved around the field
- Created a repeatable software interface between field position and mechanical shot configuration

### X-Mode Defensive Control

One challenge we encountered during competition was **defensive robots contacting our chassis while we were preparing to shoot**. Even a small amount of pushing or rotation could change the robot’s orientation enough to move the shot off target. To address this, I implemented an automatic **X-mode** that activates when the robot is preparing to shoot. X-mode commands the drivetrain wheels into an X configuration, creating resistance against external forces and making it significantly harder for another robot to push or rotate our chassis. This allowed us to stabilize the robot during the critical moment of the shot without requiring the Driver to manually activate a defensive mode.

I also designed X-mode to remain **fully responsive to Driver input**. Rather than locking the drivetrain for a fixed amount of time, the system continuously monitors the Driver’s movement commands. The moment the Driver moves the joystick to intentionally reposition the robot, X-mode automatically disengages and returns control of the drivetrain to the Driver. This created a balance between **automatic shot stabilization and human control**: the software protected our shooting position when we were stationary, but never prevented the Driver from reacting to defensive pressure or changing position when necessary.

**Impact:**  
- Reduced unwanted drivetrain rotation during shots  
- Helped maintain shooting alignment when challenged by defensive robots  
- Removed the need for the Driver to manually activate and deactivate the lock  
- Preserved immediate Driver control whenever movement was commanded  
- Improved the shooter’s robustness against unpredictable defensive contact  

### Shooter Readiness Validation

A major challenge in rapid-cycle scoring was ensuring that the shooter was actually ready before feeding a game piece.

I implemented a **readiness check** that compares the measured flywheel velocity and hood position against their calculated targets. The subsystem uses different tolerances for hub and shuttle shots to account for their different operating requirements.

The shooter is considered ready only when **both the flywheel and hood are within their required tolerances**.

**Implementation Details:**  
- **Flywheel validation:** Compares the slowest measured flywheel speed against the calculated target
- **Hood validation:** Compares the measured hood position against the calculated target angle
- **Mode-dependent tolerances:** Uses tighter tolerances for hub shots and separate tolerances for shuttle shots
- **System-level validation:** Combines individual mechanism checks into a single shooter readiness condition
- **Dashboard feedback:** Logs the individual readiness conditions for debugging and driver awareness

**Impact:**  
- Prevented game pieces from being launched before the shooter reached its required configuration
- Improved shot consistency
- Reduced reliance on Driver timing
- Provided clear subsystem diagnostics during testing and competition

## Reflection

Working on the shooter subsystem taught me how to connect **robot localization, closed-loop motor control, physics, and mechanical behavior** into a single system. Instead of treating the shooter as simply a pair of motors, I had to consider how distance, robot motion, hood position, flywheel velocity, and projectile behavior interact to determine whether a shot would be successful.

Implementing distance-based interpolation and shooter readiness validation reinforced the importance of converting experimental data into reliable software behavior.

This project strengthened my ability to develop control systems that combine **real-time sensor data, mathematical models, experimentally tuned parameters, and closed-loop control** to produce predictable behavior in a dynamic competition environment.

## Acknowledgments
- Team Valor 6800
- CTRE Phoenix Library
- WPILib
- Eigen