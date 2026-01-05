
package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.meet2.M2_CompTeleop;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "M3_CompTeleop", group = "Competition")
public class M3_TeleOp extends OpMode {

    // Mecanum Wheels
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    // Intake/Launcher
    DcMotorEx launcher;
    DcMotor intake1, intake2;
    Servo armServo;
    ElapsedTime timer;

    //double forwardBackward;
    //double strafeRightLeft;
    //double rotate = 0;
    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;

    public static M2_CompTeleop.Params PARAMS = new M2_CompTeleop.Params();

    public GoBildaPinpointDriver pinpoint;

    M3_StaticCommonFunctions scuff = new M3_StaticCommonFunctions();

    boolean toggle = false; //this checks if a has been toggled
    boolean prevButton;
    boolean last = false; //checks the value given the last time a was pressed

    static boolean shootBallsRunning = false;

    @Override
    public void init() {

        // initializing stuff
        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftDrive = hardwareMap.get(DcMotor.class, "leftBack");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightBack");

        // Intakes, Servo,  and Launcher
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        armServo = hardwareMap.get(Servo.class, "armServo");

        initAprilTag();
        timer = new ElapsedTime();

        // reverse the left motors just because that's how it works
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidf = new PIDFCoefficients(50, 0.75, 1.0, 12.7);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        // Pinpoint
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, ConfigurationConstants.Names.ODOMETRY_COMPUTER);

        double mmPerTick = PARAMS.inPerTick * 25.4;
        pinpoint.setEncoderResolution(1 / mmPerTick, DistanceUnit.MM);
        pinpoint.setOffsets(mmPerTick * PARAMS.parYTicks, mmPerTick * PARAMS.perpXTicks, DistanceUnit.MM);

        //Encoder Directions
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        //double mmPerTick = PARAMS.inPerTick * 25.4;
        pinpoint.setEncoderResolution(1 / mmPerTick, DistanceUnit.MM);
        pinpoint.setOffsets(mmPerTick * PARAMS.parYTicks, mmPerTick * PARAMS.perpXTicks, DistanceUnit.MM);
        //Encoder Directions
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        pinpoint.resetPosAndIMU();
    }
    @Override
    public void start(){
        launcher.setVelocity(1380);
    }

    @Override
    public void loop() {

        pinpoint.update();

        boolean currentButton;


        // INTAKE AND LAUNCHER CODE BELOW
        intake1.setPower(1.0);
        //intake2.setPower(-1.0);

        //Servo Bind

        if (gamepad2.x) {
            armServo.setPosition(0);
        }
        // if left bumper isn't pressed, intake 2 won't run
        else {
            armServo.setPosition(1);
        }
        if (gamepad1.a){
            pinpoint.resetPosAndIMU();
        }

        /* if (gamepad1.left_bumper) {
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

         */

        currentButton = gamepad1.b;
        if (currentButton && !prevButton && timer.seconds() > 0.3) {
            toggle = !toggle;
            timer.reset();
        }
        prevButton = currentButton;
        // depending on the value (true or false) it will go to either robot centric or field centric
        if (toggle) {
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }


        if (gamepad2.left_bumper) {
            intake2.setPower(-1.0);
        }
        // if right trigger is pressed, it will launch the artifacts
        if (gamepad2.right_trigger > 0) {
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            //Calling shooter method that should dynamically set the velocity based on Apriltag detection
            scuff.shootBallAprilTagDistance(launcher, intake1, intake2, armServo, aprilTag, this);

        } else if (gamepad2.right_bumper) {
            launcher.setVelocity(1800);
        }
        if (!shootBallsRunning){
            intake2.setPower(0);
        }
        // Emergency Brake
        if (gamepad2.b) {
            launcher.setVelocity(0);
            intake1.setPower(0);
            intake2.setPower(0);
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);
            armServo.setPosition(1);
            try {
                Thread.sleep(3000);
            } catch (InterruptedException e) {
                // needed this to not have warning
            }
        }
        else {
            launcher.setVelocity(1300);
            intake1.setPower(1);
        }

    }

    public AprilTagDetection detectAprilTag (int tag, List<AprilTagDetection> currentDetections ){

        // Step through the list of detected tags and look for a matching tag
        AprilTagDetection dummyTag = new AprilTagDetection(-1, -1 , 1.900F, null, null, null, null, null, null, 123);

        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((detection.id == tag)) {
                    // Yes, we want to use this tag.
                    telemetry.addData("\n\n\n\n", "TAG FOUND!!!!!!!");
                    return detection;
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }
        return dummyTag;
    }

    public void drive(double forwardBackward, double strafeRightLeft, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forwardBackward + strafeRightLeft + rotate;
        double frontRightPower = forwardBackward - strafeRightLeft - rotate;
        double backRightPower = forwardBackward + strafeRightLeft - rotate;
        double backLeftPower = forwardBackward - strafeRightLeft + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }
    private void driveFieldRelative(double forwardBackward, double strafeRightLeft, double rotate) {

        telemetry.addLine("Hold left bumper to drive in robot relative");
        telemetry.addLine("The left joystick sets the robot direction");
        telemetry.addLine("Moving the right joystick left and right turns the robot");
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forwardBackward, strafeRightLeft);
        double r = Math.hypot(strafeRightLeft, forwardBackward);
        double pinpoint_Heading = pinpoint.getHeading(AngleUnit.RADIANS);
        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                pinpoint_Heading);

        // Third, convert back to cartesian
        double newForwardBackward = r * Math.sin(theta);
        double newStrafeRightLeft = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForwardBackward, newStrafeRightLeft, rotate);

    }
    public void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }
}
