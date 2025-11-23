package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name="Field Centric TeleOp Tutorial")
//@Disabled


public class FieldCentricDriveTutorial extends OpMode {
    GoBildaPinpointDriver odo; // Declare Odometry Computer
    private DcMotor BLeft;
    private DcMotor BRight;
    private DcMotor FLeft;
    private DcMotor FRight;

    @Override
    public void init() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        FLeft = hardwareMap.get(DcMotor.class, "left_front");
        FRight = hardwareMap.get(DcMotor.class, "right_front");
        BLeft = hardwareMap.get(DcMotor.class, "left_back");
        BRight = hardwareMap.get(DcMotor.class, "right_back");

        // reverse the motor diretions
        BLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Odometry Computer Configuration
        odo.setOffsets(83.0,-115.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Odometry Starting Position
        odo.resetPosAndIMU();
        Pose2D startingPosition= new Pose2D(DistanceUnit.MM, -923.925, 1681.47, AngleUnit.RADIANS, 0);
        odo.setPosition(startingPosition);

        telemetry.addData("Status","Initialized");
        //telemetry.addData("X offset", odo.getXOffset());
        //telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();
    }


    public void moveRobot(){

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        Pose2D pos = odo.getPosition();
        double heading = pos.getHeading(AngleUnit.RADIANS);

        double cosAngle = Math.cos((Math.PI / 2)-heading);
        double sinAngle = Math.sin((Math.PI / 2)-heading);

        double globalStrafe = -forward * sinAngle + strafe * cosAngle;
        double globalForward = forward * cosAngle + strafe * sinAngle;

        double[] newWheelSpeeds = new double[4];

        newWheelSpeeds[0] = globalForward + globalStrafe + rotate;
        newWheelSpeeds[1] = globalForward - globalStrafe - rotate;
        newWheelSpeeds[2] = globalForward - globalStrafe + rotate;
        newWheelSpeeds[3] = globalForward + globalStrafe - rotate;

        FLeft.setPower(newWheelSpeeds[0]);
        FRight.setPower(newWheelSpeeds[1]);
        BLeft.setPower(newWheelSpeeds[2]);
        BRight.setPower(newWheelSpeeds[3]);
        telemetry.addData("Robot XPos: ", pos.getX(DistanceUnit.MM));
        telemetry.addData("Robot YPos: ", pos.getY(DistanceUnit.MM));
        telemetry.addData("Robot Heading: ", heading);
        telemetry.addData("Forward Speed : ", globalForward);
        telemetry.addData("Strafe Speed : ", globalStrafe);

        telemetry.addData("Forward Speed : ", globalForward);
        telemetry.addData("Strafe Speed : ", globalStrafe);

    }

    @Override
    public void loop() {
        moveRobot();

        Pose2D pos = odo.getPosition();
        telemetry.addData("Robot X: ", odo.getPosX(DistanceUnit.MM));
        telemetry.addData("Robot X: ", odo.getPosY(DistanceUnit.MM));
        telemetry.addData("Robot X: ", pos.getHeading(AngleUnit.DEGREES));
        telemetry.update();


    }
}