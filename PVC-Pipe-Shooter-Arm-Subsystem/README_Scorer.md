# PVC Pipe Shooter + Arm Subsystem – FRC Team Valor 6800 (2025)

Software for the scoring mechanism of Team Valor 6800’s 2025 FIRST Robotics Competition (FRC) robot, designed to intake, control, and score game pieces reliably.

## Project Overview & Scope

This repository contains the scoring subsystem code that I designed, implemented, and tested **with another student** for Team Valor 6800’s 2025 competition robot. The robot competed in multiple Texas District events and advanced to the FIRST Championship, where scoring reliability and fault tolerance were critical to match success.

Due to team intellectual property and collaboration policies, this repository does **not** include the full robot project and will not build or run independently. Video demonstrations and photos showing the subsystem operating on the physical robot are provided in my portfolio.

## Subsystem Objective

The scoring subsystem is responsible for intaking, transferring, and scoring two distinct types of game pieces: a **cylindrical PVC pipe** and a **large rubber ball**, across multiple heights and field locations.

**My Design Constraints:**  
- Reliably handle two geometrically different game pieces  
- Detect when the robot has acquired a game piece  
- Coordinate multiple motors and sensors during handoff between mechanisms  
- Remain responsive to Driver and Operator input under match time pressure  
- Allow quick recovery from misaligned or partially intaked game pieces, jams, double-feeds, or accidental ejection  
- Support rapid scoring cycles without requiring precise driver alignment  

## Control Architecture

### State-Based Control

The scoring subsystem is implemented as a **supervisory, event-driven state machine** that coordinates the elevator, pivot, intake, and funnel based on Driver/Operator intent, drivetrain alignment, and sensor feedback (e.g., beambreaks, current sensors, and CANrange devices).

Rather than running a fully scripted scoring routine, the system uses **human-initiated, conditionally autonomous control**: the Driver selects a height and initiates the scoring sequence, and the software manages alignment validation, elevator positioning, and game-piece release once all safety and accuracy conditions are satisfied.

### Single Responsibility Principle

I structured this subsystem using the **Single Responsibility Principle**, where each state manages a specific set of parameters and behaviors. The system is a **Mealy state machine**, where outputs (elevator movement, pivot angles, intake/funnel speed) respond directly to both the current state and real-time sensor inputs.

**Advantages of this design:**  
- **Modularity:** Each state can be developed, tested, and debugged independently  
- **Extensibility:** New game pieces or scoring strategies can be added with minimal impact on existing code  
- **Reliability:** Safety-critical decisions are consistently enforced within dedicated states  
- **Responsiveness:** Real-time sensor feedback ensures immediate reaction to field variability  
- **Efficient coordination:** Multiple motors and mechanisms operate in synchronized handoff sequences, reducing cycle time and improving scoring consistency  

### Pivoting Arm

After winning the State Championship, our team reviewed match footage and noticed that **ground pickup and low-level scoring** were becoming increasingly important.

The design and mechanical team built a **pivoting arm** that could mount onto our existing scoring mechanism just before the World Championship. I had one week to integrate this hardware, so I designed and implemented a separate state machine that interfaced cleanly with the existing subsystem.

**My Design Constraints:**  
- Move safely and reliably in all scenarios  
- Guide PVC pipe into the shooter when intaking
- Grab rubber balls off the structure  
- Avoid collisions  
- Support conditional behavior based on **elevator position** and **Driver input**
- Preserve the system's previous functions (don't compromise the accuracy, speed etc our old mechanism had)

**Implementation Details:**  
- **Safe movement:** Programmed state-dependent safety checks to prevent collisions with existing mechanisms (elevator, drivetrain, climber, etc.) and field obstacles  
- **Pipe intake guidance:** Default state has the arm resting on top of the shooter guiding PVC pipes effectively
- **Rubber ball pickup:** Arm extends in a dedicated state when Driver/Operator indicates they want to pick up a rubber ball  
- **State-dependent behavior:** Elevator position and Driver/Operator input determine which state the arm enters, controlling motion safely  

**Impact:**  
- Added ground pickup and low-level scoring capability  
- Improved intake efficiency  
- Preserved reliability of the existing scoring subsystem  

### Auto Dunk

After our first competition, my teammate and I analyzed robot logs and discovered a consistent **250–500 ms delay** between when the chassis was correctly positioned to score and when the gamepiece was released. We diagnosed that this latency was caused by **human reaction time**, rather than mechanical or software limitations.

To address this, we implemented **Auto Dunk**, a conditionally autonomous scoring feature that:  
- Reduces scoring delay and Driver workload  
- Preserves Driver authority  

Rather than relying on the Driver to manually time the release, the subsystem continuously evaluates a set of **gating conditions** before executing the final scoring action, including:  
- Drivetrain alignment confidence  
- Elevator positional accuracy  
- Game piece type  
- Driver intent (through button presses)  
- Sensor validity  

The Driver signals scoring intent, and the software executes the release **immediately once all spatial and safety constraints are met**, preventing delayed or misaligned shots under match pressure.

**Impact:**  
- Improved scoring consistency  
- Reduced cycle time from **12 s → 8 s**  
- Enabled real-time adaptation to field variability  
- Reinforced the value of human-in-the-loop, conditionally autonomous control  

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
## Reflection

Working on the scoring subsystem taught me the importance of balancing human control with adaptive automation. I learned to anticipate variability on the field, misaligned game pieces, jams, and driver timing, and design software that could respond in real time while preserving operator authority. Implementing Auto Dunk and integrating the pivoting arm reinforced the value of modular design, iterative testing, and safety-critical state management, showing me that even small timing improvements or state transitions can significantly enhance performance. This project strengthened my ability to coordinate complex mechanisms, diagnose issues, and build robust, responsive systems that perform reliably in dynamic, high-pressure environments.

## Acknowledgments
- Team Valor 6800 
- CTRE Phoenix Library
- WPILib

