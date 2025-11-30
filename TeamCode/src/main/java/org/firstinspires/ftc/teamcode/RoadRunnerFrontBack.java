package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.util.BasicDiagnosticFormatter;

import org.firstinspires.ftc.ftccommon.internal.AnnotatedHooksClassFilter;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class RoadRunnerFrontBack extends LinearOpMode {

    // Distance thresholds
    private static final double ROBOT_DETECTION_CM = 30.0;
    private static final double BLOCKED_TIMEOUT_SEC = 2.0;
    private static final double WALL_DETECTION_CM = 15.0;

    // Movement parameters
    private static final double DRIVE_POWER = 0.5;
    private static final double AVOIDANCE_ANGLE = 90;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Road Runner drive with starting localizer.getPose()
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(new Vector2d(0, 0), 0));

        // Initialize distance sensors
        DistanceSensor frontSensor = hardwareMap.get(DistanceSensor.class, "frontDistance");
        DistanceSensor backSensor = hardwareMap.get(DistanceSensor.class, "backDistance");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Example: Drive forward 60 inches with obstacle avoidance
        driveForwardWithAvoidance(drive, frontSensor, 60, DRIVE_POWER);

        // Example: Drive backward with obstacle avoidance
        // driveBackwardWithAvoidance(drive, backSensor, 60, DRIVE_POWER);

        telemetry.addData("Status", "Autonomous Complete");
        telemetry.update();
    }

    private void driveForwardWithAvoidance(MecanumDrive drive,
                                           DistanceSensor sensor,
                                           double targetDistanceInches,
                                           double power) {
        drive.localizer.update();
        Pose2d startPose = drive.localizer.getPose();
        ElapsedTime blockedTimer = new ElapsedTime();
        boolean isBlocked = false;
        double distanceTraveled = 0;

        while (opModeIsActive() && distanceTraveled < targetDistanceInches) {
            double currentDistance = sensor.getDistance(DistanceUnit.CM);

            if (currentDistance <= ROBOT_DETECTION_CM && currentDistance > WALL_DETECTION_CM) {
                if (!isBlocked) {
                    blockedTimer.reset();
                    isBlocked = true;
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

                    telemetry.addData("Status", "Obstacle detected! Waiting...");
                    telemetry.addData("Distance (cm)", currentDistance);
                    telemetry.update();
                }

                if (blockedTimer.seconds() >= BLOCKED_TIMEOUT_SEC) {
                    telemetry.addData("Status", "Blocked too long - going around");
                    telemetry.update();

                    performAvoidanceManeuver(drive, sensor, true);

                    isBlocked = false;
                    drive.localizer.update();
                    startPose = drive.localizer.getPose();
                }
            } else if (currentDistance <= WALL_DETECTION_CM) {
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                telemetry.addData("Status", "Wall detected - stopping");
                telemetry.addData("Distance (cm)", currentDistance);
                telemetry.update();
                break;
            } else {
                if (isBlocked) {
                    telemetry.addData("Status", "Path cleared - continuing");
                    telemetry.update();
                }
                isBlocked = false;

                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(power, 0), 0));

                telemetry.addData("Status", "Driving forward");
                telemetry.addData("Distance (cm)", currentDistance);
                telemetry.addData("Distance traveled (in)", distanceTraveled);
                telemetry.update();
            }

            drive.updatePoseEstimate();
            drive.localizer.update();
            Pose2d currentPose = drive.localizer.getPose();
            distanceTraveled = Math.hypot(
                    currentPose.position.x - startPose.position.x,
                    currentPose.position.y - startPose.position.y
            );
        }

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        telemetry.addData("Status", "Forward drive complete");
        telemetry.update();
        sleep(500);
    }

    private void driveBackwardWithAvoidance(MecanumDrive drive,
                                            DistanceSensor sensor,
                                            double targetDistanceInches,
                                            double power) {
        drive.localizer.update();

        Pose2d startPose = drive.localizer.getPose();
        ElapsedTime blockedTimer = new ElapsedTime();
        boolean isBlocked = false;
        double distanceTraveled = 0;

        while (opModeIsActive() && distanceTraveled < targetDistanceInches) {
            double currentDistance = sensor.getDistance(DistanceUnit.CM);

            if (currentDistance <= ROBOT_DETECTION_CM && currentDistance > WALL_DETECTION_CM) {
                if (!isBlocked) {
                    blockedTimer.reset();
                    isBlocked = true;
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

                    telemetry.addData("Status", "Obstacle behind! Waiting...");
                    telemetry.addData("Distance (cm)", currentDistance);
                    telemetry.update();
                }

                if (blockedTimer.seconds() >= BLOCKED_TIMEOUT_SEC) {
                    telemetry.addData("Status", "Blocked too long - going around");
                    telemetry.update();

                    performAvoidanceManeuver(drive, sensor, false);

                    isBlocked = false;
                    drive.localizer.update();

                    startPose = drive.localizer.getPose();
                }
            } else if (currentDistance <= WALL_DETECTION_CM) {
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                telemetry.addData("Status", "Wall detected - stopping");
                telemetry.update();
                break;
            } else {
                if (isBlocked) {
                    telemetry.addData("Status", "Path cleared - continuing");
                    telemetry.update();
                }
                isBlocked = false;

                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-power, 0), 0));

                telemetry.addData("Status", "Driving backward");
                telemetry.addData("Distance (cm)", currentDistance);
                telemetry.addData("Distance traveled (in)", distanceTraveled);
                telemetry.update();
            }

            drive.updatePoseEstimate();
            drive.localizer.update();


            Pose2d currentPose = drive.localizer.getPose();
            distanceTraveled = Math.hypot(
                    currentPose.position.x - startPose.position.x,
                    currentPose.position.y - startPose.position.y
            );
        }

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        telemetry.addData("Status", "Backward drive complete");
        telemetry.update();
        sleep(500);
    }

    private void performAvoidanceManeuver(MecanumDrive drive,
                                          DistanceSensor sensor,
                                          boolean isForward) {
        telemetry.addData("Avoidance", "Step 1: Rotating 90 degrees");
        telemetry.update();

        // Step 1: Rotate 90 degrees to the right
        drive.localizer.update();

        double targetHeading = drive.localizer.getPose().heading.toDouble() + Math.toRadians(AVOIDANCE_ANGLE);

        while (opModeIsActive() && Math.abs(drive.localizer.getPose().heading.toDouble() - targetHeading) > Math.toRadians(5)) {
            double headingError = targetHeading - drive.localizer.getPose().heading.toDouble();
            double turnPower = Math.signum(headingError) * 0.3;
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), turnPower));
            drive.updatePoseEstimate();
        }
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

        sleep(300);

        // Step 2: Drive forward to get past obstacle
        telemetry.addData("Avoidance", "Step 2: Driving around");
        telemetry.update();

        ElapsedTime driveTimer = new ElapsedTime();
        driveTimer.reset();
        while (opModeIsActive() && driveTimer.seconds() < 2.0) {
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(DRIVE_POWER, 0), 0));
            drive.updatePoseEstimate();
        }

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        sleep(300);

        // Step 3: Rotate back to original heading
        telemetry.addData("Avoidance", "Step 3: Returning to path");
        telemetry.update();
        drive.localizer.update();


        targetHeading = drive.localizer.getPose().heading.toDouble() - Math.toRadians(AVOIDANCE_ANGLE);

        while (opModeIsActive() && Math.abs(drive.localizer.getPose().heading.toDouble() - targetHeading) > Math.toRadians(5)) {
            double headingError = targetHeading - drive.localizer.getPose().heading.toDouble();
            double turnPower = Math.signum(headingError) * 0.3;
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), turnPower));
            drive.updatePoseEstimate();
        }
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

        sleep(300);

        telemetry.addData("Avoidance", "Complete!");
        telemetry.update();
        sleep(500);
        
    }
}
