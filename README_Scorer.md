# 2025 FRC Team 6800 Robot – Scoring Subsystem

Software for the scoring mechanism of Team Valor 6800’s 2025 FIRST Robotics Competition (FRC) robot, designed to intake, control, and score game pieces reliably.

## Project Overview & Scope

This repository contains the scoring subsystem code that I designed, implemented, and tested **with another student** for Team Valor 6800’s 2025 competition robot. The robot competed in multiple Texas District events and advanced to the FIRST Championship, where scoring reliability and fault tolerance were critical to match success.

Due to team intellectual property and collaboration policies, this repository does **not** include the full robot project and will not build or run independently. Video demonstrations and photos showing the climber operating on the physical robot are provided in my MIT Maker Portfolio.

## Subsystem Objective

The scoring subsystem is responsible for intaking, transferring, and scoring 2 types of gamepieces (a cylindrical PVC pipe and a large rubber ball) across multiple heights and field locations.

My design constraints:

- Reliably handle two geometrically different gamepieces
- Be able to tell when the robot has acquired a gamepiece
- Coordinate multiple motors and sensors during handoff between mechanisms
- Remain responsive to Driver and Operator input under match time pressure
- Allow quick recovery from misaligned or partially intaked gamepieces, jams, double-feeds, and accidental ejection
- Support rapid scoring cycles without requiring precise driver alignment

## Control Architecture

### State-Based Control

The scoring subsystem is implemented as a supervisory, event-driven state machine that coordinates the elevator, pivot, intake, and funnel based on Driver/Operator intent, drivetrain alignment, and sensor feedback (e.g., beambreaks, current sensors, and CANrange devices). 

Rather than running a fully scripted scoring routine, the system uses human-initiated, conditionally autonomous control: the Driver selects a height and initiates the scoring sequence, and the software autonomously manages alignment validation, elevator positioning, and game-piece release once all safety and accuracy conditions are satisfied.
### Single Responsibility Principle

I chose to structure this subsystem using the Single Responsibility Principle: each state manages a specific set of parameters and behaviors, making the system modular, maintainable, and easy to extend. Conceptually, the system behaves like a Mealy machine, where outputs (elevator movement, pivot angles, intake/funnel speed) respond directly to both the current state and real-time sensor inputs, rather than purely on state alone.

This design provides several advantages:

- **Modularity:** Each state can be developed, tested, and debugged independently, isolating complexity and reducing unintended interactions.
- **Extensibility:** New game pieces or scoring strategies can be added with minimal impact on existing code, enabling rapid iteration and scaling.
- **Reliability:** Safety-critical decisions are consistently enforced within dedicated states.
- **Responsiveness:** Real-time sensor feedback ensures the system reacts immediately to field variability.
- **Efficient coordination:** Multiple motors and mechanisms operate in synchronized handoff sequences, reducing cycle time and improving scoring consistency.

### Auto Dunk

Auto Dunk is a conditionally autonomous scoring feature designed to reduce driver workload while preserving the Driver’s authority over when scoring occurs. 

After our first competition, my teammate and I analyzed our robot logs and saw a consistent 250–500 millisecond delay between the moment the robot chassis was correctly positioned to score and when the gamepiece was released. We determined that this latency was caused by human reaction time rather than mechanical or software limitations.

With Auto Dunk, rather than relying on the Driver to time the release manually, the subsystem continuously evaluates a set of gating conditions, including drivetrain alignment confidence, elevator positional accuracy, game piece type, scoped state, and sensor validity, before autonomously committing to the final scoring action. The Driver signals scoring intent, but the software executes the moment all spatial and safety constraints are satisfied, preventing delayed shots and misaligned releases under match pressure. This approach improves scoring consistency and cycle time in tight field tolerances while avoiding rigid scripted sequences, allowing the robot to adapt in real time to field variability.



## Acknowledgments

- Team Valor 6800 
- CTRE Phoenix Library
- WPILib


