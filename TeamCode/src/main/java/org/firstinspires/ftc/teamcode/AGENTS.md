# Gearcheologists Robot Software Documentation

This document describes the autonomous agents, teleoperated modes, and libraries used by the Gearcheologists robot.

## Libraries & Frameworks
- **RoadRunner v1.0.1**: Used for advanced path planning and trajectory execution in autonomous modes.
- **AprilTag (FTC Vision)**: Integrated for real-time localization and target alignment. Used to calculate precise distances for autonomous and teleop shooting.
- **GoBilda Pinpoint Driver**: High-performance odometry system providing precise field-centric coordinates and heading.
- **FTC Dashboard**: Utilized for real-time telemetry visualization and parameter tuning.

## Autonomous Agents
The autonomous routines are designed to maximize points by navigating to scoring zones and using vision-assisted shooting.

### Red Side
- **RedGoal_V3_Optimized.java**: The flagship autonomous routine for the Red Goal side. Features optimized RoadRunner splines and AprilTag-based alignment for multi-shot scoring sequences.
- **RedLoadingBig/SmallTriangle.java**: Routines for navigating from the loading zone to scoring positions in the field's triangular regions.
- **RedGoal_AprilTag.java**: A specialized routine demonstrating pure AprilTag-based navigation and stabilization.

### Blue Side
- **BlueGoal.java**: Primary scoring routine for the Blue side.
- **BlueLoadingBig/SmallTriangle.java**: Counterpart routines for the Blue alliance loading zones.

### Common Infrastructure
- **Auto_CommonFunctions.java**: Contains shared logic for camera initialization, autonomous shooting sequences, and tag detection.
- **PinpointLocalizer.java**: Bridges the GoBilda Pinpoint system with RoadRunner for accurate pose estimation.

## TeleOp Modes
TeleOp modes focus on driver efficiency through automation and field-centric control.

### Primary Modes
- **CompTeleOp.java**: The main competition driver interface.
  - **Field-Centric Drive**: Uses the Pinpoint odometry system to maintain driver-relative controls regardless of robot orientation.
  - **Auto-Aim Shooting**: When the trigger is pulled, the robot automatically adjusts launcher velocity based on the detected AprilTag's range.
  - **Visual Feedback**: Real-time LED status (Green = Locked, Red = Searching) for AprilTag target acquisition.
- **BaseTeleOp.java**: Abstract foundation for all TeleOp variants, managing hardware initialization, drive logic, and safety hooks.

### Special Features
- **Field Relative Toggle**: Switch between robot-centric and field-centric driving modes.
- **Emergency Stop**: Dedicated hardware interrupt (GamePad 2 B button) to immediately neutralize all actuators.
- **Variable Intake**: Intelligent intake management that pauses during shooting to prevent jams.
