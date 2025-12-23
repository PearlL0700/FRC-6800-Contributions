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

### Pivoting Arm

After winning the State Championship, our team reviewed match footage and noticed that ground pickup and low-level scoring were becoming more important. In response, the design and mechanical team built a pivoting arm that could mount onto our existing scoring mechanism just before the World Championship. I had one week to integrate this hardware, so I designed and implemented a separate state machine that interfaced cleanly with the existing subsystem.

The main challenge was ensuring the arm moved safely and correctly in all scenarios. I programmed state-dependent rules so it would guide pipe intake by default, extend only for the rubber ball, and retract afterward to avoid collisions, with conditional overrides based on elevator position and Driver input. This integration added ground pickup and low-level scoring, improved intake efficiency, while preserving the reliable scoring functions from States.

### Auto Dunk

After our first competition, my teammate and I analyzed robot logs and noticed a consistent 250–500 ms delay between when the chassis was correctly positioned to score and when the gamepiece was released. We determined that this latency was due to human reaction time rather than mechanical or software limitations.

To address this, we implemented Auto Dunk, a conditionally autonomous scoring feature that reduces scoring delay and Driver workload while preserving Driver authority. Rather than relying on the Driver to manually time the release, the subsystem continuously evaluates a set of gating conditions, including drivetrain alignment confidence, elevator positional accuracy, game piece type, scoped state, and sensor validity, before executing the final scoring action. The Driver signals scoring intent, and the software executes it immediately once all spatial and safety constraints are met, preventing delayed or misaligned shots under match pressure.

This approach improved scoring consistency, reduced cycle time from an average of 12 s to 8 s, and allowed the robot to adapt in real time to field variability.

The following snippet illustrates how the subsystem evaluates multiple sensor and state conditions before committing to the scoring action:

```cpp
  if (
        (state.autoDunkEnabled && !disableAutoDunk) &&
        state.scopedState == SCOPED_STATE::SCOPED &&
        elevatorWithinThreshold &&
        state.gamePiece == GAME_PIECE::CORAL &&
        (state.elevState == TWO || state.elevState == THREE || state.elevState == FOUR) &&
        drivetrain->getAutoDunkAcceptance().all()
    ) {
        state.scoringState = SCORE_STATE::SCORING;
    }
```
## Acknowledgments

- Team Valor 6800 
- CTRE Phoenix Library
- WPILib



