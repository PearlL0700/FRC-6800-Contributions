# Intake Subsystem – FRC Team Valor 6800 (2026)

Software for the intaking mechanism of Team Valor 6800’s 2026 FIRST Robotics Competition (FRC) robot, designed to intake and control game pieces reliably.

## Project Overview & Scope

This repository contains the intake subsystem code that I designed, implemented, and tested **with another student** for Team Valor 6800’s 2026 competition robot. The robot competed in multiple Texas District events and advanced to the FIRST Championship, where scoring reliability and fault tolerance were critical to match success.

Due to team intellectual property and collaboration policies, this repository does **not** include the full robot project and will not build or run independently. Video demonstrations and photos showing the subsystem operating on the physical robot are provided in my portfolio.

## Subsystem Objective

The intake subsystem is responsible for reliably acquiring a **dense foam ball** from the carpet across multiple field locations and transferring it into the robot for scoring.

**My Design Constraints:**  
- Reliably handle the game piece
- Detect when the mechanism has jammed on the game piece
- Remain responsive to Driver and Operator input under match time pressure  
- Allow quick recovery from misaligned or partially intaked game pieces
- Support rapid intaking cycles without requiring precise driver alignment
- Agitate the game pieces to allow them to rapidly exit the robot when scoring

## Control Architecture

### State-Based Control

The intake subsystem is implemented as a **supervisory, event-driven state machine** that coordinates the intake, feeder, hopper, and pivot based on Driver/Operator input and real-time system feedback.

Rather than relying on continuous manual control of each motor, the system uses defined states for behaviors such as **intaking, outtaking, shooting, and disabling** the mechanism. Driver/Operator inputs determine the desired subsystem behavior, while the software coordinates the individual motors and mechanisms to execute that behavior.

The intake, feeder, hopper, and pivot states independently determine their respective motor outputs while working together as a coordinated subsystem.

The system also incorporates **real-time current monitoring** to detect when the intake mechanism has stalled on a game piece. When a jam is detected, the software provides immediate haptic feedback to the Driver, allowing them to respond without relying on visual inspection.

**Advantages of this design:**  
- **Modularity:** Each mechanism can be developed, tested, and debugged independently  
- **Reliability:** Dedicated states provide predictable behavior for each operating mode  
- **Responsiveness:** Driver and Operator inputs directly control subsystem behavior during competition  
- **Fault Detection:** Motor current monitoring identifies potential intake jams in real time  
- **Efficient coordination:** Intake, feeder, and hopper motors operate in coordinated sequences to rapidly move game pieces through the robot  
- **Maintainability:** Separating mechanism behavior into independent states makes the subsystem easier to modify and troubleshoot  

### Intake Agitation

To increase the rate at which game pieces could be transferred to the shooter, I implemented a **shimmy sequence** that rapidly alternates the intake pivot between two positions. The sequence uses timed instant commands to repeatedly move the intake in and out, agitating the foam balls and helping them exit the robot faster.

**Implementation Details:**  
- **Timed sequencing:** Alternates between `SHIMMY_IN` and `SHIMMY_OUT` states with a configurable `SHIMMY_INTERVAL`
- **Pivot control:** Uses instant commands to change the intake pivot state without introducing unnecessary command overhead
- **Game-piece agitation:** Rapidly moves the intake to shift foam balls toward the shooter

**Impact:**  
- Increased the rate at which game pieces could exit the robot by 4 balls per second
- Reduced delays between consecutive shots
- Improved overall scoring cycle speed

**Code snippet:**
```cpp
frc2::CommandPtr Intake::shimmyIntake() {
    return frc2::SequentialCommandGroup(
               frc2::InstantCommand([this]() { state.pivotState = PIVOT_STATE::SHIMMY_IN; }), frc2::WaitCommand(SHIMMY_INTERVAL),
               frc2::InstantCommand([this]() { state.pivotState = PIVOT_STATE::SHIMMY_OUT; }), frc2::WaitCommand(SHIMMY_INTERVAL))
        .ToPtr();
}
```

## Acknowledgments
- Team Valor 6800 
- CTRE Phoenix Library
- WPILib

