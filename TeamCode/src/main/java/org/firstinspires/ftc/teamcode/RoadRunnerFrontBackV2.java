/*
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="RoadRunner FrontBack V2")
public class RoadRunnerFrontBackV2 extends LinearOpMode {

    // Distance thresholds
    private static final double ROBOT_DETECTION_CM = 35.0;
    private static final double BLOCKED_TIMEOUT_SEC = 2.0;
    private static final double WALL_DETECTION_CM = 8.0;

    // Movement parameters
    private static final double DRIVE_POWER = 0.5;
    private static final double AVOIDANCE_DISTANCE_INCHES = 24.0;
    private static final double AVOIDANCE_ANGLE = 90;
    private static final double ROTATION_TOLERANCE = 2.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Set telemetry to log mode to keep all messages
        telemetry.log().setCapacity(50);
        telemetry.setMsTransmissionInterval(50);

        telemetry.log().add("=== INITIALIZATION ===");

        // Initialize Road Runner drive
        telemetry.log().add("Initializing RoadRunner drive...");
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(new Vector2d(0, 0), 0));
        telemetry.log().add("✓ Drive initialized");

        // Initialize distance sensors
        telemetry.log().add("Initializing distance sensors...");
        DistanceSensor frontSensor = hardwareMap.get(DistanceSensor.class, "frontDistance");
        DistanceSensor backSensor = hardwareMap.get(DistanceSensor.class, "backDistance");
        telemetry.log().add("✓ Front sensor: " + frontSensor.getDeviceName());
        telemetry.log().add("✓ Back sensor: " + backSensor.getDeviceName());

        telemetry.addData("Status", "Ready to Start");
        telemetry.log().add("=== PRESS START ===");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.log().add("=== AUTONOMOUS STARTED ===");
        telemetry.log().add("Target: 120 inches forward");
        telemetry.update();

        // Drive forward 120 inches with obstacle avoidance
        driveForwardWithAvoidance(drive, frontSensor, 120, DRIVE_POWER);

        telemetry.log().add("=== AUTONOMOUS COMPLETE ===");
        telemetry.addData("Status", "FINISHED");
        telemetry.update();
    }

    private void driveForwardWithAvoidance(MecanumDrive drive,
                                           DistanceSensor sensor,
                                           double targetDistanceInches,
                                           double power) {
        Pose2d startPose = drive.localizer.getPose();
        ElapsedTime blockedTimer = new ElapsedTime();
        ElapsedTime totalTimer = new ElapsedTime();
        boolean isBlocked = false;
        double distanceTraveled = 0;
        int loopCount = 0;

        telemetry.log().add("--- Starting Forward Drive ---");
        telemetry.log().add(String.format("Start Pose: X=%.1f, Y=%.1f, Heading=%.1f°",
                startPose.position.x, startPose.position.y, Math.toDegrees(startPose.heading.toDouble())));

        while (opModeIsActive() && distanceTraveled < targetDistanceInches) {
            loopCount++;

            // Update pose estimate
            drive.updatePoseEstimate();

            // Calculate distance traveled
            Pose2d currentPose = drive.localizer.getPose();
            distanceTraveled = Math.hypot(
                    currentPose.position.x - startPose.position.x,
                    currentPose.position.y - startPose.position.y
            );

            double currentDistance = sensor.getDistance(DistanceUnit.CM);

            // Check for obstacles (but ignore walls)
            if (currentDistance <= ROBOT_DETECTION_CM && currentDistance > WALL_DETECTION_CM) {
                if (!isBlocked) {
                    blockedTimer.reset();
                    isBlocked = true;
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

                    telemetry.log().add("⚠ OBSTACLE DETECTED!");
                    telemetry.log().add(String.format("Sensor: %.1f cm", currentDistance));
                    telemetry.log().add(String.format("At position: X=%.1f, Y=%.1f",
                            currentPose.position.x, currentPose.position.y));
                }

                // Current status display
                telemetry.addData("⏸ STATUS", "BLOCKED - WAITING");
                telemetry.addData("Wait Time", "%.1f / %.1f sec", blockedTimer.seconds(), BLOCKED_TIMEOUT_SEC);
                telemetry.addData("Sensor Distance", "%.1f cm", currentDistance);
                telemetry.addData("Position", "X=%.1f Y=%.1f", currentPose.position.x, currentPose.position.y);
                telemetry.addData("Distance Done", "%.1f / %.1f in", distanceTraveled, targetDistanceInches);
                telemetry.addData("Loop Count", loopCount);
                telemetry.update();

                // If blocked for too long, go around
                if (blockedTimer.seconds() >= BLOCKED_TIMEOUT_SEC) {
                    telemetry.log().add("🔄 TIMEOUT - Starting Avoidance");
                    telemetry.update();

                    performAvoidanceManeuver(drive, sensor, true);

                    isBlocked = false;
                    startPose = drive.localizer.getPose();
                    distanceTraveled = 0;
                    telemetry.log().add("✓ Avoidance Complete - Resuming");
                    telemetry.log().add(String.format("New Start: X=%.1f, Y=%.1f",
                            startPose.position.x, startPose.position.y));
                }
            } else {
                // Path is clear, drive forward
                if (isBlocked) {
                    telemetry.log().add("✓ Path Clear - Resuming Drive");
                    isBlocked = false;
                }

                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(power, 0), 0));

                // Current status display
                telemetry.addData("▶ STATUS", "DRIVING FORWARD");
                telemetry.addData("Power", "%.2f", power);
                telemetry.addData("Progress", "%.1f / %.1f in (%.0f%%)",
                        distanceTraveled, targetDistanceInches, (distanceTraveled/targetDistanceInches)*100);
                telemetry.addData("Sensor Distance", "%.1f cm ✓", currentDistance);
                telemetry.addData("Current Pose", "X=%.1f Y=%.1f", currentPose.position.x, currentPose.position.y);
                telemetry.addData("Heading", "%.1f°", Math.toDegrees(currentPose.heading.toDouble()));
                telemetry.addData("Runtime", "%.1f sec", totalTimer.seconds());
                telemetry.addData("Loop Count", loopCount);
                telemetry.update();
            }

            sleep(50); // Small delay for loop stability
        }

        // Stop when target reached
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

        Pose2d finalPose = drive.localizer.getPose();
        telemetry.log().add("✓ Target Reached!");
        telemetry.log().add(String.format("Final: X=%.1f, Y=%.1f, Heading=%.1f°",
                finalPose.position.x, finalPose.position.y, Math.toDegrees(finalPose.heading.toDouble())));
        telemetry.log().add(String.format("Total Distance: %.1f inches", distanceTraveled));
        telemetry.log().add(String.format("Total Time: %.1f seconds", totalTimer.seconds()));
        telemetry.log().add(String.format("Total Loops: %d", loopCount));
        telemetry.update();
    }

    private void driveBackwardWithAvoidance(MecanumDrive drive,
                                            DistanceSensor sensor,
                                            double targetDistanceInches,
                                            double power) {
        Pose2d startPose = drive.localizer.getPose();
        ElapsedTime blockedTimer = new ElapsedTime();
        ElapsedTime totalTimer = new ElapsedTime();
        boolean isBlocked = false;
        double distanceTraveled = 0;
        int loopCount = 0;

        telemetry.log().add("--- Starting Backward Drive ---");
        telemetry.log().add(String.format("Start Pose: X=%.1f, Y=%.1f, Heading=%.1f°",
                startPose.position.x, startPose.position.y, Math.toDegrees(startPose.heading.toDouble())));

        while (opModeIsActive() && distanceTraveled < targetDistanceInches) {
            loopCount++;
            drive.updatePoseEstimate();

            Pose2d currentPose = drive.localizer.getPose();
            distanceTraveled = Math.hypot(
                    currentPose.position.x - startPose.position.x,
                    currentPose.position.y - startPose.position.y
            );

            double currentDistance = sensor.getDistance(DistanceUnit.CM);

            if (currentDistance <= ROBOT_DETECTION_CM && currentDistance > WALL_DETECTION_CM) {
                if (!isBlocked) {
                    blockedTimer.reset();
                    isBlocked = true;
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

                    telemetry.log().add("⚠ OBSTACLE DETECTED (REAR)!");
                    telemetry.log().add(String.format("Sensor: %.1f cm", currentDistance));
                }

                telemetry.addData("⏸ STATUS", "BLOCKED - WAITING");
                telemetry.addData("Wait Time", "%.1f / %.1f sec", blockedTimer.seconds(), BLOCKED_TIMEOUT_SEC);
                telemetry.addData("Sensor Distance", "%.1f cm", currentDistance);
                telemetry.addData("Distance Done", "%.1f / %.1f in", distanceTraveled, targetDistanceInches);
                telemetry.update();

                if (blockedTimer.seconds() >= BLOCKED_TIMEOUT_SEC) {
                    telemetry.log().add("🔄 TIMEOUT - Starting Avoidance");
                    performAvoidanceManeuver(drive, sensor, false);

                    isBlocked = false;
                    startPose = drive.localizer.getPose();
                    distanceTraveled = 0;
                    telemetry.log().add("✓ Avoidance Complete");
                }
            } else {
                if (isBlocked) {
                    telemetry.log().add("✓ Path Clear - Resuming");
                    isBlocked = false;
                }

                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-power, 0), 0));

                telemetry.addData("◀ STATUS", "DRIVING BACKWARD");
                telemetry.addData("Power", "-%.2f", power);
                telemetry.addData("Progress", "%.1f / %.1f in (%.0f%%)",
                        distanceTraveled, targetDistanceInches, (distanceTraveled/targetDistanceInches)*100);
                telemetry.addData("Sensor Distance", "%.1f cm ✓", currentDistance);
                telemetry.addData("Runtime", "%.1f sec", totalTimer.seconds());
                telemetry.addData("Loop Count", loopCount);
                telemetry.update();
            }

            sleep(50);
        }

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        telemetry.log().add("✓ Backward Drive Complete!");
        telemetry.log().add(String.format("Distance: %.1f inches in %.1f sec", distanceTraveled, totalTimer.seconds()));
        telemetry.update();
    }

    private void performAvoidanceManeuver(MecanumDrive drive, DistanceSensor sensor, boolean isForward) {
        telemetry.log().add("╔══════════════════════════╗");
        telemetry.log().add("║  AVOIDANCE MANEUVER      ║");
        telemetry.log().add("╚══════════════════════════╝");

        Pose2d startPose = drive.localizer.getPose();
        telemetry.log().add(String.format("Start: X=%.1f, Y=%.1f, H=%.1f°",
                startPose.position.x, startPose.position.y, Math.toDegrees(startPose.heading.toDouble())));

        // Step 1: Rotate 90 degrees (right)
        telemetry.log().add("Step 1: Rotating 90° right...");
        telemetry.addData("🔄 AVOIDANCE", "Rotating Right 90°");
        telemetry.update();
        rotateRobot(drive, AVOIDANCE_ANGLE);
        sleep(200);
        telemetry.log().add("✓ Rotation complete");

        // Step 2: Drive sideways to go around obstacle
        telemetry.log().add(String.format("Step 2: Moving sideways %.1f inches...", AVOIDANCE_DISTANCE_INCHES));
        telemetry.addData("🔄 AVOIDANCE", "Moving Sideways");
        telemetry.update();
        driveStraight(drive, AVOIDANCE_DISTANCE_INCHES, DRIVE_POWER);
        sleep(200);
        telemetry.log().add("✓ Sideways move complete");

        // Step 3: Rotate back to original heading
        telemetry.log().add("Step 3: Rotating back...");
        telemetry.addData("🔄 AVOIDANCE", "Rotating Back");
        telemetry.update();
        rotateRobot(drive, -AVOIDANCE_ANGLE);
        sleep(200);
        telemetry.log().add("✓ Back to heading");

        // Step 4: Drive forward past the obstacle
        telemetry.log().add(String.format("Step 4: Passing obstacle (%.1f in)...", AVOIDANCE_DISTANCE_INCHES));
        telemetry.addData("🔄 AVOIDANCE", "Passing Obstacle");
        telemetry.update();
        driveStraight(drive, AVOIDANCE_DISTANCE_INCHES, DRIVE_POWER);
        sleep(200);
        telemetry.log().add("✓ Obstacle passed");

        // Step 5: Rotate to face original direction
        telemetry.log().add("Step 5: Rotating left...");
        telemetry.addData("🔄 AVOIDANCE", "Rotating Left");
        telemetry.update();
        rotateRobot(drive, -AVOIDANCE_ANGLE);
        sleep(200);
        telemetry.log().add("✓ Facing back");

        // Step 6: Drive back to centerline
        telemetry.log().add(String.format("Step 6: Returning to path (%.1f in)...", AVOIDANCE_DISTANCE_INCHES));
        telemetry.addData("🔄 AVOIDANCE", "Returning to Path");
        telemetry.update();
        driveStraight(drive, AVOIDANCE_DISTANCE_INCHES, DRIVE_POWER);
        sleep(200);
        telemetry.log().add("✓ Back on centerline");

        // Step 7: Rotate back to forward orientation
        telemetry.log().add("Step 7: Final rotation...");
        telemetry.addData("🔄 AVOIDANCE", "Final Alignment");
        telemetry.update();
        rotateRobot(drive, AVOIDANCE_ANGLE);
        sleep(200);

        Pose2d endPose = drive.localizer.getPose();
        telemetry.log().add("✓ AVOIDANCE COMPLETE!");
        telemetry.log().add(String.format("End: X=%.1f, Y=%.1f, H=%.1f°",
                endPose.position.x, endPose.position.y, Math.toDegrees(endPose.heading.toDouble())));
        telemetry.log().add(String.format("Net Movement: dX=%.1f, dY=%.1f",
                endPose.position.x - startPose.position.x,
                endPose.position.y - startPose.position.y));
        telemetry.update();
    }

    private void rotateRobot(MecanumDrive drive, double targetDegrees) {
        Pose2d startPose = drive.localizer.getPose();
        double startHeading = Math.toDegrees(startPose.heading.toDouble());
        double targetHeading = startHeading + targetDegrees;
        ElapsedTime rotationTimer = new ElapsedTime();
        int updateCount = 0;

        telemetry.log().add(String.format("Rotating %.1f° (from %.1f° to %.1f°)",
                targetDegrees, startHeading, targetHeading));

        while (opModeIsActive()) {
            updateCount++;
            drive.updatePoseEstimate();
            Pose2d currentPose = drive.localizer.getPose();
            double currentHeading = Math.toDegrees(currentPose.heading.toDouble());
            double headingError = targetHeading - currentHeading;

            // Normalize error to -180 to 180
            while (headingError > 180) headingError -= 360;
            while (headingError < -180) headingError += 360;

            if (Math.abs(headingError) < ROTATION_TOLERANCE) {
                telemetry.log().add(String.format("✓ Rotation complete: %.1f° (error: %.2f°)",
                        currentHeading, headingError));
                break;
            }

            double rotationPower = Math.signum(headingError) * 0.3;
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), rotationPower));

            // Update display every 5 iterations to reduce lag
            if (updateCount % 5 == 0) {
                telemetry.addData("🔄 ROTATING", targetDegrees > 0 ? "RIGHT" : "LEFT");
                telemetry.addData("Target Heading", "%.1f°", targetHeading);
                telemetry.addData("Current Heading", "%.1f°", currentHeading);
                telemetry.addData("Error", "%.2f°", headingError);
                telemetry.addData("Power", "%.2f", rotationPower);
                telemetry.addData("Time", "%.2f sec", rotationTimer.seconds());
                telemetry.update();
            }

            sleep(20);
        }

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }

    private void driveStraight(MecanumDrive drive, double distanceInches, double power) {
        Pose2d startPose = drive.localizer.getPose();
        double distanceTraveled = 0;
        ElapsedTime driveTimer = new ElapsedTime();
        int updateCount = 0;

        telemetry.log().add(String.format("Driving straight: %.1f inches at %.2f power", distanceInches, power));

        while (opModeIsActive() && distanceTraveled < distanceInches) {
            updateCount++;
            drive.updatePoseEstimate();
            Pose2d currentPose = drive.localizer.getPose();
            distanceTraveled = Math.hypot(
                    currentPose.position.x - startPose.position.x,
                    currentPose.position.y - startPose.position.y
            );

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(power, 0), 0));

            // Update display every 5 iterations
            if (updateCount % 5 == 0) {
                telemetry.addData("➤ DRIVING STRAIGHT", "");
                telemetry.addData("Progress", "%.1f / %.1f in (%.0f%%)",
                        distanceTraveled, distanceInches, (distanceTraveled/distanceInches)*100);
                telemetry.addData("Power", "%.2f", power);
                telemetry.addData("Position", "X=%.1f Y=%.1f", currentPose.position.x, currentPose.position.y);
                telemetry.addData("Time", "%.2f sec", driveTimer.seconds());
                telemetry.update();
            }

            sleep(20);
        }

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

        Pose2d finalPose = drive.localizer.getPose();
        telemetry.log().add(String.format("✓ Straight drive complete: %.1f inches in %.2f sec",
                distanceTraveled, driveTimer.seconds()));
        telemetry.log().add(String.format("Final: X=%.1f, Y=%.1f", finalPose.position.x, finalPose.position.y));
    }
}*/

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="RoadRunner FrontBack V2")
public class RoadRunnerFrontBackV2 extends LinearOpMode {

    // Distance thresholds
    private static final double ROBOT_DETECTION_CM = 35.0;
    private static final double BLOCKED_TIMEOUT_SEC = 2.0;
    private static final double WALL_DETECTION_CM = 8.0;

    // Movement parameters
    private static final double DRIVE_POWER = 0.5;
    private static final double AVOIDANCE_DISTANCE_INCHES = 24.0;
    private static final double AVOIDANCE_ANGLE = 90;
    private static final double ROTATION_TOLERANCE = 2.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Set telemetry to log mode to keep all messages
        telemetry.log().setCapacity(50);
        telemetry.setMsTransmissionInterval(50);

        telemetry.log().add("=== INITIALIZATION ===");

        // Initialize Road Runner drive
        telemetry.log().add("Initializing RoadRunner drive...");
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(new Vector2d(0, 0), 0));
        telemetry.log().add("✓ Drive initialized");

        // Initialize distance sensors
        telemetry.log().add("Initializing distance sensors...");
        DistanceSensor frontSensor = hardwareMap.get(DistanceSensor.class, "frontDistance");
        DistanceSensor backSensor = hardwareMap.get(DistanceSensor.class, "backDistance");
        telemetry.log().add("✓ Front sensor: " + frontSensor.getDeviceName());
        telemetry.log().add("✓ Back sensor: " + backSensor.getDeviceName());

        telemetry.addData("Status", "Ready to Start");
        telemetry.log().add("=== PRESS START ===");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.log().add("=== AUTONOMOUS STARTED ===");
        telemetry.log().add("Target: 120 inches forward");
        telemetry.update();

        // Drive forward 120 inches with obstacle avoidance
        driveForwardWithAvoidance(drive, frontSensor, 120, DRIVE_POWER);

        telemetry.log().add("=== AUTONOMOUS COMPLETE ===");
        telemetry.addData("Status", "FINISHED");
        telemetry.update();
    }

    private void driveForwardWithAvoidance(
            MecanumDrive drive,
            DistanceSensor sensor,
            double targetDistanceInches,
            double power
    ) {
        Pose2d startPose = drive.localizer.getPose();
        ElapsedTime blockedTimer = new ElapsedTime();
        ElapsedTime totalTimer = new ElapsedTime();
        boolean isBlocked = false;
        double distanceTraveled = 0;
        int loopCount = 0;

        telemetry.log().add("--- Starting Forward Drive ---");
        telemetry.log().add(String.format(
                "Start Pose: X=%.1f, Y=%.1f, Heading=%.1f°",
                startPose.position.x,
                startPose.position.y,
                Math.toDegrees(startPose.heading.toDouble())
        ));

        while (opModeIsActive() && distanceTraveled < targetDistanceInches) {
            loopCount++;

            // Update pose estimate
            drive.updatePoseEstimate();

            // Calculate distance traveled
            Pose2d currentPose = drive.localizer.getPose();
            distanceTraveled = Math.hypot(
                    currentPose.position.x - startPose.position.x,
                    currentPose.position.y - startPose.position.y
            );

            double currentDistance = sensor.getDistance(DistanceUnit.CM);

            // Check for obstacles (ignore walls)
            if (currentDistance <= ROBOT_DETECTION_CM && currentDistance > WALL_DETECTION_CM) {

                // --- BLOCKED STATE ---
                if (!isBlocked) {
                    blockedTimer.reset();
                    isBlocked = true;

                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

                    telemetry.log().add("⚠ OBSTACLE DETECTED!");
                    telemetry.log().add(String.format("Sensor: %.1f cm", currentDistance));
                    telemetry.log().add(String.format("At position: X=%.1f, Y=%.1f",
                            currentPose.position.x, currentPose.position.y));
                }

                telemetry.addData("⏸ STATUS", "BLOCKED - WAITING");
                telemetry.addData("Wait Time", "%.1f / %.1f sec",
                        blockedTimer.seconds(), BLOCKED_TIMEOUT_SEC);
                telemetry.addData("Sensor Distance", "%.1f cm", currentDistance);
                telemetry.addData("Position", "X=%.1f Y=%.1f",
                        currentPose.position.x, currentPose.position.y);
                telemetry.addData("Distance Done", "%.1f / %.1f in",
                        distanceTraveled, targetDistanceInches);
                telemetry.addData("Loop Count", loopCount);
                telemetry.update();

                // If blocked too long → avoidance
                if (blockedTimer.seconds() >= BLOCKED_TIMEOUT_SEC) {

                    telemetry.log().add("🔄 TIMEOUT - Starting Avoidance");
                    telemetry.update();

                    performAvoidanceManeuver(drive, sensor, true);

                    isBlocked = false;
                    startPose = drive.localizer.getPose();
                    distanceTraveled = 0;

                    telemetry.log().add("✓ Avoidance Complete - Resuming");
                    telemetry.log().add(String.format("New Start: X=%.1f, Y=%.1f",
                            startPose.position.x, startPose.position.y));
                }

            } else {

                // --- PATH CLEAR ---
                if (isBlocked) {
                    telemetry.log().add("✓ Path Clear - Resuming Drive");
                    isBlocked = false;
                }

                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(power, 0), 0));

                telemetry.addData("▶ STATUS", "DRIVING FORWARD");
                telemetry.addData("Power", "%.2f", power);
                telemetry.addData("Progress", "%.1f / %.1f in (%.0f%%)",
                        distanceTraveled, targetDistanceInches,
                        (distanceTraveled / targetDistanceInches) * 100);
                telemetry.addData("Sensor Distance", "%.1f cm ✓", currentDistance);
                telemetry.addData("Current Pose", "X=%.1f Y=%.1f",
                        currentPose.position.x, currentPose.position.y);
                telemetry.addData("Heading", "%.1f°",
                        Math.toDegrees(currentPose.heading.toDouble()));
                telemetry.addData("Runtime", "%.1f sec", totalTimer.seconds());
                telemetry.addData("Loop Count", loopCount);
                telemetry.update();
            }

            sleep(50);
        }

        // Stop when done
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

        Pose2d finalPose = drive.localizer.getPose();

        telemetry.log().add("✓ Target Reached!");
        telemetry.log().add(String.format(
                "Final: X=%.1f, Y=%.1f, Heading=%.1f°",
                finalPose.position.x,
                finalPose.position.y,
                Math.toDegrees(finalPose.heading.toDouble())
        ));
        telemetry.log().add(String.format("Total Distance: %.1f inches", distanceTraveled));
        telemetry.log().add(String.format("Total Time: %.1f seconds", totalTimer.seconds()));
        telemetry.log().add(String.format("Total Loops: %d", loopCount));
        telemetry.update();
    }

    private void driveBackwardWithAvoidance(
            MecanumDrive drive,
            DistanceSensor sensor,
            double targetDistanceInches,
            double power
    ) {
        Pose2d startPose = drive.localizer.getPose();
        ElapsedTime blockedTimer = new ElapsedTime();
        ElapsedTime totalTimer = new ElapsedTime();
        boolean isBlocked = false;
        double distanceTraveled = 0;
        int loopCount = 0;

        telemetry.log().add("--- Starting Backward Drive ---");
        telemetry.log().add(String.format(
                "Start Pose: X=%.1f, Y=%.1f, Heading=%.1f°",
                startPose.position.x,
                startPose.position.y,
                Math.toDegrees(startPose.heading.toDouble())
        ));

        while (opModeIsActive() && distanceTraveled < targetDistanceInches) {
            loopCount++;

            drive.updatePoseEstimate();
            Pose2d currentPose = drive.localizer.getPose();
            distanceTraveled = Math.hypot(
                    currentPose.position.x - startPose.position.x,
                    currentPose.position.y - startPose.position.y
            );

            double currentDistance = sensor.getDistance(DistanceUnit.CM);

            if (currentDistance <= ROBOT_DETECTION_CM && currentDistance > WALL_DETECTION_CM) {

                // --- BLOCKED ---
                if (!isBlocked) {
                    blockedTimer.reset();
                    isBlocked = true;
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

                    telemetry.log().add("⚠ OBSTACLE DETECTED (REAR)!");
                    telemetry.log().add(String.format("Sensor: %.1f cm", currentDistance));
                }

                telemetry.addData("⏸ STATUS", "BLOCKED - WAITING");
                telemetry.addData("Wait Time", "%.1f / %.1f sec",
                        blockedTimer.seconds(), BLOCKED_TIMEOUT_SEC);
                telemetry.addData("Sensor Distance", "%.1f cm", currentDistance);
                telemetry.addData("Distance Done", "%.1f / %.1f in",
                        distanceTraveled, targetDistanceInches);
                telemetry.update();

                if (blockedTimer.seconds() >= BLOCKED_TIMEOUT_SEC) {
                    telemetry.log().add("🔄 TIMEOUT - Starting Avoidance");

                    performAvoidanceManeuver(drive, sensor, false);

                    isBlocked = false;
                    startPose = drive.localizer.getPose();
                    distanceTraveled = 0;

                    telemetry.log().add("✓ Avoidance Complete");
                }

            } else {

                // --- CLEAR ---
                if (isBlocked) {
                    telemetry.log().add("✓ Path Clear - Resuming");
                    isBlocked = false;
                }

                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-power, 0), 0));

                telemetry.addData("◀ STATUS", "DRIVING BACKWARD");
                telemetry.addData("Power", "-%.2f", power);
                telemetry.addData("Progress", "%.1f / %.1f in (%.0f%%)",
                        distanceTraveled, targetDistanceInches,
                        (distanceTraveled / targetDistanceInches) * 100);
                telemetry.addData("Sensor Distance", "%.1f cm ✓", currentDistance);
                telemetry.addData("Runtime", "%.1f sec", totalTimer.seconds());
                telemetry.addData("Loop Count", loopCount);
                telemetry.update();
            }

            sleep(50);
        }

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

        telemetry.log().add("✓ Backward Drive Complete!");
        telemetry.log().add(String.format(
                "Distance: %.1f inches in %.1f sec",
                distanceTraveled, totalTimer.seconds()
        ));
        telemetry.update();
    }

    private void performAvoidanceManeuver(
            MecanumDrive drive,
            DistanceSensor sensor,
            boolean isForward
    ) {
        telemetry.log().add("╔══════════════════════════╗");
        telemetry.log().add("║  AVOIDANCE MANEUVER      ║");
        telemetry.log().add("╚══════════════════════════╝");

        Pose2d startPose = drive.localizer.getPose();

        telemetry.log().add(String.format(
                "Start: X=%.1f, Y=%.1f, H=%.1f°",
                startPose.position.x,
                startPose.position.y,
                Math.toDegrees(startPose.heading.toDouble())
        ));

        // Step 1: Rotate 90° right
        telemetry.log().add("Step 1: Rotating 90° right...");
        telemetry.addData("🔄 AVOIDANCE", "Rotating Right 90°");
        telemetry.update();
        rotateRobot(drive, AVOIDANCE_ANGLE);
        sleep(200);
        telemetry.log().add("✓ Rotation complete");

        // Step 2: Move sideways
        telemetry.log().add(String.format(
                "Step 2: Moving sideways %.1f inches...",
                AVOIDANCE_DISTANCE_INCHES
        ));
        telemetry.addData("🔄 AVOIDANCE", "Moving Sideways");
        telemetry.update();
        driveStraight(drive, AVOIDANCE_DISTANCE_INCHES, DRIVE_POWER);
        sleep(200);
        telemetry.log().add("✓ Sideways move complete");

        // Step 3: Rotate back
        telemetry.log().add("Step 3: Rotating back...");
        telemetry.addData("🔄 AVOIDANCE", "Rotating Back");
        telemetry.update();
        rotateRobot(drive, -AVOIDANCE_ANGLE);
        sleep(200);
        telemetry.log().add("✓ Back to heading");

        // Step 4: Move forward past obstacle
        telemetry.log().add(String.format(
                "Step 4: Passing obstacle (%.1f in)...",
                AVOIDANCE_DISTANCE_INCHES
        ));
        telemetry.addData("🔄 AVOIDANCE", "Passing Obstacle");
        telemetry.update();
        driveStraight(drive, AVOIDANCE_DISTANCE_INCHES, DRIVE_POWER);
        sleep(200);
        telemetry.log().add("✓ Obstacle passed");

        // Step 5: Rotate left
        telemetry.log().add("Step 5: Rotating left...");
        telemetry.addData("🔄 AVOIDANCE", "Rotating Left");
        telemetry.update();
        rotateRobot(drive, -AVOIDANCE_ANGLE);
        sleep(200);
        telemetry.log().add("✓ Facing back");

        // Step 6: Move sideways back to center
        telemetry.log().add(String.format(
                "Step 6: Returning to path (%.1f in)...",
                AVOIDANCE_DISTANCE_INCHES
        ));
        telemetry.addData("🔄 AVOIDANCE", "Returning to Path");
        telemetry.update();
        driveStraight(drive, AVOIDANCE_DISTANCE_INCHES, DRIVE_POWER);
        sleep(200);
        telemetry.log().add("✓ Back on centerline");

        // Step 7: Final rotation
        telemetry.log().add("Step 7: Final rotation...");
        telemetry.addData("🔄 AVOIDANCE", "Final Alignment");
        telemetry.update();
        rotateRobot(drive, AVOIDANCE_ANGLE);
        sleep(200);

        Pose2d endPose = drive.localizer.getPose();

        telemetry.log().add("✓ AVOIDANCE COMPLETE!");
        telemetry.log().add(String.format(
                "End: X=%.1f, Y=%.1f, H=%.1f°",
                endPose.position.x,
                endPose.position.y,
                Math.toDegrees(endPose.heading.toDouble())
        ));
        telemetry.log().add(String.format(
                "Net Movement: dX=%.1f, dY=%.1f",
                endPose.position.x - startPose.position.x,
                endPose.position.y - startPose.position.y
        ));
        telemetry.update();
    }

    private void rotateRobot(MecanumDrive drive, double targetDegrees) {
        Pose2d startPose = drive.localizer.getPose();
        double startHeading = Math.toDegrees(startPose.heading.toDouble());
        double targetHeading = startHeading + targetDegrees;

        ElapsedTime rotationTimer = new ElapsedTime();
        int updateCount = 0;

        telemetry.log().add(String.format(
                "Rotating %.1f° (from %.1f° to %.1f°)",
                targetDegrees,
                startHeading,
                targetHeading
        ));

        while (opModeIsActive()) {
            updateCount++;

            drive.updatePoseEstimate();
            Pose2d currentPose = drive.localizer.getPose();
            double currentHeading = Math.toDegrees(currentPose.heading.toDouble());

            double headingError = targetHeading - currentHeading;

            // Normalize -180 to 180
            while (headingError > 180) headingError -= 360;
            while (headingError < -180) headingError += 360;

            if (Math.abs(headingError) < ROTATION_TOLERANCE) {
                telemetry.log().add(String.format(
                        "✓ Rotation complete: %.1f° (error: %.2f°)",
                        currentHeading,
                        headingError
                ));
                break;
            }

            double rotationPower = Math.signum(headingError) * 0.3;

            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(0, 0),
                            rotationPower
                    )
            );
        }

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }

    private void driveStraight(
            MecanumDrive drive,
            double distanceInches,
            double power
    ) {
        Pose2d startPose = drive.localizer.getPose();
        double distanceMoved = 0;

        while (opModeIsActive() && distanceMoved < distanceInches) {

            drive.updatePoseEstimate();
            Pose2d currentPose = drive.localizer.getPose();

            distanceMoved = Math.hypot(
                    currentPose.position.x - startPose.position.x,
                    currentPose.position.y - startPose.position.y
            );

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(power, 0), 0));

            telemetry.addData("Driving Straight", "%.1f / %.1f in",
                    distanceMoved, distanceInches);
            telemetry.update();

            sleep(20);
        }

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }
}
