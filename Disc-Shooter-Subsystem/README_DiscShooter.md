# Disc Shooter Subsystem – FRC Team Valor 6800 (2024)

Software for the shooting mechanism of Team Valor 6800’s 2024 FIRST Robotics Competition (FRC) robot, designed to automatically configure, stage, and launch game pieces reliably across varying distances.

## Project Overview & Scope

This repository contains the shooter subsystem code that I designed, implemented, and tested **with another student** for Team Valor 6800’s 2024 competition robot. The robot competed in multiple Texas District events and advanced to the FIRST Championship, where shot consistency, rapid scoring cycles, and reliable game-piece handling were critical to match success.

The shooter subsystem consists of several coordinated mechanisms, including the **flywheels, pivot, internal intake rollers, and feeder**. Together, these mechanisms control the trajectory of the game piece, stage it within the robot, and shoot the game piece when initiated. 

Due to team intellectual property and collaboration policies, this repository does **not** include the full robot project and will not build or run independently. Video demonstrations and photos showing the subsystem operating on the physical robot are provided in my portfolio.

## Subsystem Objective

The shooter subsystem is responsible for launching a **foam ring** into scoring targets across varying field locations and heights. The system automatically determines the required shooter configuration based on the robot's position relative to the target while coordinating the flywheels, pivot, intake, and feeder to stage and launch the game piece.

**My Design Constraints:**  
- Reliably score game pieces across varying distances  
- Automatically determine appropriate flywheel speed and pivot angle based on distance to the target  
- Maintain consistent shot parameters as the robot moves around the field  
- Detect and stage game pieces within the shooter before launching  
- Coordinate multiple motors during game-piece transfer and shooting  
- Prevent overfeeding or incorrectly positioning multiple game pieces  
- Verify that the shooter has reached its required operating conditions before shooting  
- Support rapid scoring cycles without requiring manual adjustment of individual mechanisms  
- Remain responsive to Driver and Operator input under match time pressure  

## Control Architecture

### State-Based Control

The shooter subsystem is implemented using **state-based control**, with independent states for the flywheel, pivot, intake rollers, and feeder. Each mechanism determines its behavior based on Driver/Operator input, robot state, target selection, and real-time sensor feedback. 

Rather than requiring the Driver to manually control individual motors, the system coordinates multiple mechanisms through predefined operating states such as **tracking, intaking, shooting, outtaking, tuning, and stopping**.

During normal operation, the shooter continuously calculates the required flywheel velocity and pivot position while the internal intake and feeder manage the position of the game piece within the robot.

### Single Responsibility Principle

I structured the shooter subsystem so that each mechanism is responsible for a specific part of the shooting process.

- The **flywheels** control projectile launch velocity
- The **pivot** controls the launch angle
- The **internal intake rollers** move the game piece toward the shooter
- The **feeder** stages and transfers the game piece into the flywheels
- **Beam-break sensors** determine the position of the game piece during transfer

Although each mechanism operates independently, their states are coordinated to create a complete shooting sequence. 

**Advantages of this design:**  
- **Modularity:** Individual mechanisms can be developed, tested, and debugged independently  
- **Reliability:** Dedicated states provide predictable behavior for each operating mode  
- **Responsiveness:** The subsystem continuously responds to Driver/Operator input and sensor feedback  
- **Efficient coordination:** Multiple motors operate together to stage and launch game pieces  
- **Maintainability:** Separating mechanism responsibilities makes the system easier to modify and troubleshoot  
- **Extensibility:** Additional shot configurations or game-piece handling behaviors can be added without redesigning the entire subsystem  

## Distance-Based Shot Control

A major challenge was allowing the robot to score from different locations without requiring the Driver to manually select a flywheel speed and pivot angle for every shot.

I implemented **a line of best fit** using experimentally determined shot parameters. The subsystem calculates the robot's distance from the selected target and interpolates between predefined shot values to determine the required flywheel velocity and pivot angle. :contentReference[oaicite:3]{index=3}

The system maintains separate shot profiles for different scoring situations, allowing the shooter to use different configurations depending on the selected target and field position.

**Implementation Details:**  
- **Distance measurement:** Calculates the distance between the robot and the selected scoring target using the robot's estimated field position  
- **Shot line of best fit:** Determines the required flywheel velocity and pivot angle from experimentally tuned tables  
- **Multiple shot profiles:** Supports different shot configurations for hub and shuttle shots  
- **Continuous adjustment:** Recalculates shooter setpoints as the robot moves relative to the target  
- **Closed-loop control:** Commands the flywheel and pivot toward their calculated setpoints  

**Impact:**  
- Reduced Driver workload when configuring shots  
- Enabled consistent shooting across a range of field positions  
- Allowed the shooter to automatically adapt to changes in robot position  
- Created a repeatable relationship between field position and shooter configuration  

## Game-Piece Detection and Staging

The shooter uses **two beam-break sensors** to track the position of a game piece as it moves through the internal intake and feeder mechanisms. The sensors allow the subsystem to automatically adjust motor behavior instead of relying entirely on Driver timing. :contentReference[oaicite:4]{index=4}

When a game piece reaches different positions within the mechanism, the intake and feeder motors respond accordingly. The system can slow, continue, or stop the motors to stage the game piece and prevent it from being overfed into the shooter. When game pieces are overfed, our shot accuracy decreases.

**Implementation Details:**  
- **Feeder detection:** A beam-break sensor detects when a game piece reaches the feeder position  
- **Staging detection:** A second beam-break sensor detects when the game piece reaches another staging position  
- **Automatic stopping:** The intake and feeder can stop when the game piece reaches the desired position  
- **Controlled transfer:** Motor speeds adjust based on which sensors are triggered  
- **Debounced transitions:** Sensor events are debounced to improve reliability and prevent unintended state changes  

**Impact:**  
- Improved consistency of game-piece positioning before shooting  
- Reduced reliance on Driver timing  
- Helped prevent overfeeding  
- Enabled automated coordination between the intake and feeder mechanisms  
- Improved accuracy of shots

## Coordinated Feeding and Shooting

The internal intake and feeder mechanisms operate as part of the shooting sequence rather than as completely separate systems.

During intake, the rollers move the game piece through the robot while the feeder adjusts its speed based on beam-break feedback. During shooting, both the internal intake and feeder transition into dedicated shooting states that transfer the staged game piece into the flywheels. :contentReference[oaicite:5]{index=5}

The code also includes named commands that allow these feeding behaviors to be triggered during autonomous routines.

**Implementation Details:**  
- **Intake mode:** Internal rollers and feeder coordinate to acquire and stage a game piece  
- **Shooting mode:** Intake rollers and feeder transition to dedicated forward velocities to transfer the game piece into the shooter  
- **Outtake mode:** Motors reverse to remove or recover from incorrectly positioned game pieces  
- **Sensor-based control:** Beam-break feedback modifies motor behavior during intake  
- **Autonomous integration:** Named commands allow feeder and intake sequences to be incorporated into autonomous routines  

**Impact:**  
- Created a coordinated path from game-piece acquisition to launch  
- Supported rapid transitions between intake and shooting  
- Reduced manual coordination required from the Driver  
- Allowed the same subsystem behaviors to be used during autonomous operation  

## Shooter Readiness Validation

Before launching a game piece, the shooter evaluates whether its mechanisms have reached their required operating conditions.

The readiness logic compares the measured flywheel velocity and pivot position against their calculated setpoints. The shooter is considered ready only when the required mechanisms are sufficiently close to their targets. :contentReference[oaicite:6]{index=6}

**Implementation Details:**  
- **Flywheel validation:** Compares measured flywheel velocity against the target velocity  
- **Pivot validation:** Compares the measured pivot position against the calculated setpoint  
- **Shot-dependent tolerances:** Different shot configurations can use different acceptable error thresholds  
- **System-level readiness:** Individual mechanism checks are combined into an overall shooter-ready condition  
- **Dashboard diagnostics:** Individual subsystem conditions are exposed for testing and debugging  

**Impact:**  
- Improved shot consistency  
- Reduced the likelihood of launching before the shooter reached its intended configuration  
- Provided clear diagnostics during testing  
- Reduced reliance on Driver timing   

## Driver Feedback and Fault Recovery

Because the Driver cannot always see the game piece once it enters the robot, the subsystem provides **haptic feedback through the Driver controller** when a game piece is detected at the feeder. :contentReference[oaicite:8]{index=8}

The subsystem also supports dedicated outtake states, allowing the Driver to reverse the intake and feeder when a game piece becomes misaligned or needs to be removed.

**Impact:**  
- Provided immediate confirmation that a game piece had reached the feeder  
- Reduced reliance on visual inspection  
- Allowed faster recovery from incorrectly positioned game pieces  
- Helped the Driver maintain awareness of the robot's internal game-piece state  

## Reflection

Working on the shooter subsystem taught me how to coordinate **closed-loop motor control, sensor feedback, robot localization, mathematical models, and mechanical systems** into a single subsystem.

The project required more than simply controlling a flywheel. I had to consider how the robot's distance from the target affected launch velocity and angle, how game pieces moved through the internal intake and feeder, and how sensor feedback could be used to automatically stage the game piece before shooting.

Implementing distance-based interpolation reinforced the importance of converting experimental data into reliable software behavior. Integrating beam-break sensors and coordinated feeding logic showed me how real-time feedback can reduce Driver workload while improving consistency. The projectile simulation and motion-compensation calculations also gave me experience translating physical models into software and using those models to reason about robot behavior.

This project strengthened my ability to design **integrated control systems that combine sensor feedback, closed-loop motor control, experimentally tuned data, and mathematical modeling** to produce reliable behavior in a dynamic competition environment.

## Acknowledgments

- Team Valor 6800
- CTRE Phoenix Library
- WPILib
- PathPlanner
- Eigen