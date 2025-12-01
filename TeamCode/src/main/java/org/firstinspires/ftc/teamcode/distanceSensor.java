package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@TeleOp
public class distanceSensor extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FRONT_LEFT_DRIVE_MOTOR);
        DcMotor frontRight = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FRONT_RIGHT_DRIVE_MOTOR);
        DcMotor backLeft = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.BACK_LEFT_DRIVE_MOTOR);
        DcMotor backRight = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.BACK_RIGHT_DRIVE_MOTOR);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()){

            if (distanceSensor.getDistance(DistanceUnit.CM) < 10 && distanceSensor.getDistance(DistanceUnit.CM) > 5) {
                frontRight.setPower(0.5);
                frontLeft.setPower(-0.5);
                backLeft.setPower(-0.5);
                backRight.setPower(0.5);
            } else if (distanceSensor.getDistance(DistanceUnit.CM) < 5) {
                frontRight.setPower(0);
                frontLeft.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
            } else {
                frontRight.setPower(0.75);
                frontLeft.setPower(-0.75);
                backLeft.setPower(-0.75);
                backRight.setPower(0.75);
            }
        }
    }
}
