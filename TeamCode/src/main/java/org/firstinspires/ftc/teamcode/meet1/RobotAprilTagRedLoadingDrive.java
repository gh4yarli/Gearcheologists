package org.firstinspires.ftc.teamcode.meet1;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ConfigurationConstants;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@Autonomous
@Disabled

public class RobotAprilTagRedLoadingDrive extends DetectAprilTag
{
    // Adjust these numbers to suit your robot.

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.

    // private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private AprilTagProcessor aprilTag = null;              // Used for managing the AprilTag detection process.
    private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpMode() {

        // Initialize the Apriltag Detection process
        //  initAprilTag();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        GoBildaPinpointDriver pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, ConfigurationConstants.Names.ODOMETRY_COMPUTER);
        //  Used to control the left front drive wheel
        DcMotor frontLeftDrive = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FRONT_LEFT_DRIVE_MOTOR);
        //  Used to control the right front drive wheel
        DcMotor frontRightDrive = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FRONT_RIGHT_DRIVE_MOTOR);
        //  Used to control the left back drive wheel
        DcMotor backLeftDrive = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.BACK_LEFT_DRIVE_MOTOR);
        //  Used to control the right back drive wheel
        DcMotor backRightDrive = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.BACK_RIGHT_DRIVE_MOTOR);
        CRServo right_feeder = hardwareMap.get(CRServo.class, ConfigurationConstants.Names.RIGHT_FEEDER_SERVO);
        CRServo left_feeder = hardwareMap.get(CRServo.class, ConfigurationConstants.Names.LEFT_FEEDER_SERVO);
        DcMotor intake = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FIRST_INTAKE_MOTOR);
        DcMotor launcher = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.LEFT_LAUNCHER_MOTOR);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Pose2d startingPose = new Pose2d(-60, -12, Math.toRadians(0));
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap,startingPose);
        Action path = mecanumDrive.actionBuilder(startingPose)
                .lineToX(-20)
                .turnTo(Math.toRadians(-30))
                .build();

        // Wait for driver to press start
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        pinpointDriver.initialize();
        waitForStart();
        if (opModeIsActive()) {
            Actions.runBlocking(new SequentialAction(path));
        }
        while (timer.milliseconds() < 7500) {
            moveRobotWithTag(24, frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive, aprilTag, 55);
        }
        moveRobot(0,0,0, frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
        if (opModeIsActive()) {
            sleep(10);
            launcher.setPower(-0.56);
            sleep(2000);
            timer.reset();
            for (byte i = 0; i < 3; i++) {
                if (i > 0) intake.setPower((0.65));
                left_feeder.setPower(1);
                right_feeder.setPower(-1);
                while (timer.milliseconds() < 525) {
                    sleep(1);
                }
                timer.reset();
                left_feeder.setPower(0);
                right_feeder.setPower(0);
                while (timer.milliseconds() < 1500) {
                    sleep(1);
                }
                timer.reset();
            }
            launcher.setPower(0);
            intake.setPower(0);
        }
    }
}