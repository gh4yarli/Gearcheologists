package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * SafeDriveAction - Wraps RoadRunner actions with obstacle detection and avoidance
 * Automatically pauses when obstacles are detected and resumes when clear
 */
public class SafeDriveAction {

    /**
     * Interface for rebuilding trajectories dynamically
     */
    public interface TrajectoryBuilder {
        Action build(MecanumDrive drive, Pose2d startPose, Pose2d targetPose);
    }

    /**
     * Run an action with obstacle detection (simple version)
     * Uses default settings: 2 second wait, 3 consecutive detections required
     *
     * @param opMode The LinearOpMode instance
     * @param action The RoadRunner action to execute
     * @param drive The MecanumDrive instance
     * @param sensor The distance sensor to monitor
     * @param obstacleThreshold Distance in CM to consider as obstacle
     * @return true if action completed, false if interrupted
     */
    public static boolean runWithObstacleDetection(
            LinearOpMode opMode,
            Action action,
            MecanumDrive drive,
            Rev2mDistanceSensor sensor,
            double obstacleThreshold) {

        return runWithObstacleDetection(opMode, action, drive, sensor,
                obstacleThreshold, 2000, 3);
    }

    /**
     * Run an action with obstacle detection (advanced version)
     *
     * @param opMode The LinearOpMode instance
     * @param action The RoadRunner action to execute
     * @param drive The MecanumDrive instance
     * @param sensor The distance sensor to monitor
     * @param obstacleThreshold Distance in CM to consider as obstacle
     * @param waitTimeMs How long to wait before rechecking (milliseconds)
     * @param requiredDetections Number of consecutive detections required
     * @return true if action completed, false if interrupted
     */
    public static boolean runWithObstacleDetection(
            LinearOpMode opMode,
            Action action,
            MecanumDrive drive,
            Rev2mDistanceSensor sensor,
            double obstacleThreshold,
            long waitTimeMs,
            int requiredDetections) {

        int pauseCount = 0;
        int consecutiveDetections = 0;

        while (opMode.opModeIsActive()) {
            // Get distance reading
            double distance = sensor.getDistance(DistanceUnit.CM);

            // Update pose estimate
            drive.updatePoseEstimate();
            Pose2d currentPose = drive.localizer.getPose();

            // Display telemetry
            opMode.telemetry.addData("Distance Sensor", "%.1f cm", distance);
            opMode.telemetry.addData("Current X", "%.1f", currentPose.position.x);
            opMode.telemetry.addData("Current Y", "%.1f", currentPose.position.y);
            opMode.telemetry.addData("Current Heading", "%.1f°", Math.toDegrees(currentPose.heading.toDouble()));
            opMode.telemetry.addData("Pause Count", pauseCount);

            // Check for valid obstacle detection
            boolean validReading = (distance >= 5 && distance <= 200);

            if (validReading && distance < obstacleThreshold) {
                consecutiveDetections++;

                if (consecutiveDetections >= requiredDetections) {
                    // OBSTACLE DETECTED - STOP AND WAIT
                    stopMotors(drive);

                    pauseCount++;
                    consecutiveDetections = 0; // Reset counter

                    opMode.telemetry.addData("STATUS", "⚠️ OBSTACLE DETECTED - PAUSING");
                    opMode.telemetry.addData("Paused at", "X: %.1f, Y: %.1f",
                            currentPose.position.x, currentPose.position.y);
                    opMode.telemetry.addData("Obstacle Distance", "%.1f cm", distance);
                    opMode.telemetry.update();

                    // Wait
                    opMode.sleep(waitTimeMs);

                    // Check again after waiting
                    double distanceAfterWait = sensor.getDistance(DistanceUnit.CM);
                    boolean validAfterWait = (distanceAfterWait >= 5 && distanceAfterWait <= 200);

                    opMode.telemetry.addData("STATUS", "Checking if clear...");
                    opMode.telemetry.addData("Distance After Wait", "%.1f cm", distanceAfterWait);
                    opMode.telemetry.update();

                    if (validAfterWait && distanceAfterWait < obstacleThreshold) {
                        // Still blocked - continue to next iteration
                        opMode.telemetry.addData("RESULT", "Still blocked, will check again");
                        opMode.telemetry.update();
                        continue;
                    } else {
                        // Path is clear - resume
                        opMode.telemetry.addData("STATUS", "✓ Path clear - Resuming");
                        opMode.telemetry.addData("Resuming from", "X: %.1f, Y: %.1f",
                                currentPose.position.x, currentPose.position.y);
                        opMode.telemetry.update();
                        opMode.sleep(500); // Brief pause before resuming
                    }
                }
            } else {
                // No obstacle or not enough consecutive detections
                consecutiveDetections = 0;
                opMode.telemetry.addData("STATUS", "Driving to target");
            }

            opMode.telemetry.update();

            // Run one iteration of the action
            boolean actionRunning = action.run(new TelemetryPacket());

            // Check if action is complete
            if (!actionRunning) {
                opMode.telemetry.addData("STATUS", "✓ REACHED TARGET");
                opMode.telemetry.addData("Final Position", "X: %.1f, Y: %.1f",
                        currentPose.position.x, currentPose.position.y);
                opMode.telemetry.addData("Total Pauses", pauseCount);
                opMode.telemetry.update();
                opMode.sleep(500);
                return true; // Action completed successfully
            }
        }

        return false; // OpMode was stopped before action completed
    }

    /**
     * Run with dynamic trajectory rebuilding (simple version)
     * If an obstacle blocks the path for too long, the trajectory is rebuilt from the current position
     * Uses default settings: 2 second wait, 3 consecutive pauses before rebuild
     *
     * @param opMode The LinearOpMode instance
     * @param drive The MecanumDrive instance
     * @param sensor The distance sensor to monitor
     * @param obstacleThreshold Distance in CM to consider as obstacle
     * @param targetPose The final target pose
     * @param trajectoryBuilder Function to rebuild trajectory
     * @return true if action completed, false if interrupted
     */
    public static boolean runWithRebuild(
            LinearOpMode opMode,
            MecanumDrive drive,
            Rev2mDistanceSensor sensor,
            double obstacleThreshold,
            Pose2d targetPose,
            TrajectoryBuilder trajectoryBuilder) {

        return runWithRebuild(opMode, drive, sensor, obstacleThreshold,
                targetPose, trajectoryBuilder, 2000, 3);
    }

    /**
     * Run with dynamic trajectory rebuilding (advanced version)
     * If an obstacle blocks the path for too long, the trajectory is rebuilt from the current position
     *
     * @param opMode The LinearOpMode instance
     * @param drive The MecanumDrive instance
     * @param sensor The distance sensor to monitor
     * @param obstacleThreshold Distance in CM to consider as obstacle
     * @param targetPose The final target pose
     * @param trajectoryBuilder Function to rebuild trajectory
     * @param waitTimeMs How long to wait before rechecking (milliseconds)
     * @param maxConsecutivePauses Number of pauses before rebuilding trajectory
     * @return true if action completed, false if interrupted
     */
    public static boolean runWithRebuild(
            LinearOpMode opMode,
            MecanumDrive drive,
            Rev2mDistanceSensor sensor,
            double obstacleThreshold,
            Pose2d targetPose,
            TrajectoryBuilder trajectoryBuilder,
            long waitTimeMs,
            int maxConsecutivePauses) {

        int pauseCount = 0;
        int consecutivePauses = 0;

        // Build initial trajectory
        drive.updatePoseEstimate();
        Pose2d currentPose = drive.localizer.getPose();
        Action currentAction = trajectoryBuilder.build(drive, currentPose, targetPose);

        while (opMode.opModeIsActive()) {
            // Get distance reading
            double distance = sensor.getDistance(DistanceUnit.CM);

            // Update pose estimate
            drive.updatePoseEstimate();
            currentPose = drive.localizer.getPose();

            // Display telemetry
            opMode.telemetry.addData("Distance Sensor", "%.1f cm", distance);
            opMode.telemetry.addData("Current Position", "X: %.1f, Y: %.1f, A: %.1f°",
                    currentPose.position.x, currentPose.position.y,
                    Math.toDegrees(currentPose.heading.toDouble()));
            opMode.telemetry.addData("Target Position", "X: %.1f, Y: %.1f, A: %.1f°",
                    targetPose.position.x, targetPose.position.y,
                    Math.toDegrees(targetPose.heading.toDouble()));
            opMode.telemetry.addData("Pause Count", pauseCount);
            opMode.telemetry.addData("Consecutive Pauses", consecutivePauses);

            // Check for valid obstacle detection
            boolean validReading = (distance >= 5 && distance <= 200);

            if (validReading && distance < obstacleThreshold) {
                // OBSTACLE DETECTED - STOP AND WAIT
                stopMotors(drive);

                pauseCount++;
                consecutivePauses++;

                opMode.telemetry.addData("STATUS", "⚠️ OBSTACLE DETECTED - PAUSING");
                opMode.telemetry.addData("Obstacle Distance", "%.1f cm", distance);
                opMode.telemetry.update();

                // Wait
                opMode.sleep(waitTimeMs);

                // Check again after waiting
                double distanceAfterWait = sensor.getDistance(DistanceUnit.CM);
                boolean validAfterWait = (distanceAfterWait >= 5 && distanceAfterWait <= 200);

                if (validAfterWait && distanceAfterWait < obstacleThreshold) {
                    // Still blocked
                    if (consecutivePauses >= maxConsecutivePauses) {
                        // Rebuild trajectory from current position
                        opMode.telemetry.addData("STATUS", "🔄 Rebuilding trajectory from current position");
                        opMode.telemetry.update();

                        drive.updatePoseEstimate();
                        currentPose = drive.localizer.getPose();
                        currentAction = trajectoryBuilder.build(drive, currentPose, targetPose);

                        consecutivePauses = 0; // Reset consecutive pause counter
                        opMode.sleep(500);
                    }
                    continue;
                } else {
                    // Path is clear - resume
                    opMode.telemetry.addData("STATUS", "✓ Path clear - Resuming");
                    opMode.telemetry.update();
                    consecutivePauses = 0; // Reset on successful resume
                    opMode.sleep(500);
                }
            } else {
                // No obstacle detected
                opMode.telemetry.addData("STATUS", "Driving to target");
                consecutivePauses = 0; // Reset when no obstacle
            }

            opMode.telemetry.update();

            // Run one iteration of the action
            boolean actionRunning = currentAction.run(new TelemetryPacket());

            // Check if action is complete
            if (!actionRunning) {
                // Check if we're close enough to the target
                double distanceToTarget = Math.hypot(
                        targetPose.position.x - currentPose.position.x,
                        targetPose.position.y - currentPose.position.y
                );

                if (distanceToTarget < 2.0) { // Within 2 inches of target
                    opMode.telemetry.addData("STATUS", "✓ REACHED TARGET");
                    opMode.telemetry.addData("Final Position", "X: %.1f, Y: %.1f",
                            currentPose.position.x, currentPose.position.y);
                    opMode.telemetry.addData("Total Pauses", pauseCount);
                    opMode.telemetry.update();
                    opMode.sleep(500);
                    return true;
                } else {
                    // Action finished but we're not at target - rebuild
                    opMode.telemetry.addData("STATUS", "Action complete but not at target, rebuilding...");
                    opMode.telemetry.update();
                    drive.updatePoseEstimate();
                    currentPose = drive.localizer.getPose();
                    currentAction = trajectoryBuilder.build(drive, currentPose, targetPose);
                    opMode.sleep(500);
                }
            }
        }

        return false; // OpMode was stopped
    }

    /**
     * Stop all drive motors
     */
    private static void stopMotors(MecanumDrive drive) {
        drive.leftFront.setPower(0);
        drive.leftBack.setPower(0);
        drive.rightBack.setPower(0);
        drive.rightFront.setPower(0);
    }

    /**
     * Custom Action wrapper that includes obstacle detection
     * This can be used with Actions.runBlocking()
     */
    public static class ObstacleAwareAction implements Action {
        private final Action baseAction;
        private final Rev2mDistanceSensor sensor;
        private final double obstacleThreshold;
        private final LinearOpMode opMode;
        private final MecanumDrive drive;

        private boolean isPaused = false;
        private long pauseStartTime = 0;
        private static final long MAX_PAUSE_TIME = 5000; // 5 seconds max pause

        public ObstacleAwareAction(Action baseAction, Rev2mDistanceSensor sensor,
                                   double obstacleThreshold, LinearOpMode opMode,
                                   MecanumDrive drive) {
            this.baseAction = baseAction;
            this.sensor = sensor;
            this.obstacleThreshold = obstacleThreshold;
            this.opMode = opMode;
            this.drive = drive;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // Check sensor
            double distance = sensor.getDistance(DistanceUnit.CM);
            boolean validReading = (distance >= 5 && distance <= 200);

            if (validReading && distance < obstacleThreshold) {
                if (!isPaused) {
                    // Just detected obstacle
                    isPaused = true;
                    pauseStartTime = System.currentTimeMillis();
                    stopMotors(drive);
                }

                long pauseDuration = System.currentTimeMillis() - pauseStartTime;

                packet.put("Status", "PAUSED - Obstacle at " + String.format("%.1f", distance) + "cm");
                packet.put("Pause Duration", pauseDuration + "ms");

                // Check if paused too long
                if (pauseDuration > MAX_PAUSE_TIME) {
                    packet.put("Status", "Obstacle timeout - stopping");
                    return false; // Stop action
                }

                return true; // Still running, just paused
            }

            // Path is clear
            if (isPaused) {
                packet.put("Status", "Resuming from pause");
                isPaused = false;
            }

            // Run base action
            return baseAction.run(packet);
        }

        @Override
        public void preview(Canvas c) {
            baseAction.preview(c);
        }
    }
}