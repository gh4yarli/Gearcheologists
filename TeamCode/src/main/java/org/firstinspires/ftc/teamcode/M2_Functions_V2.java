package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public abstract class M2_Functions_V2 extends LinearOpMode {
    public void startLaunchers(DcMotorEx leftLauncher, DcMotorEx rightLauncher, double launchVelocity) {
        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DcMotorEx.Direction LeftLauncherDirection = leftLauncher.getDirection();
        DcMotorEx.Direction RightLauncherDirection = rightLauncher.getDirection();
        leftLauncher.setDirection(DcMotorEx.Direction.REVERSE);
        rightLauncher.setDirection(DcMotorEx.Direction.REVERSE);
        leftLauncher.setVelocity(launchVelocity);
        rightLauncher.setVelocity(-launchVelocity);
    }

    /**
     * This method shoots 3 balls.
     * @param leftLauncher
     * left launcher motor
     * @param rightLauncher
     * right launcher motor
     * @param leftFeeder
     * left feeder servo
     * @param rightFeeder
     * right feeder servo
     * @param intake1
     * first intake
     * @param intake2
     * second intake
     * @param launchVelocity
     * power to shoot balls
     */
    public void shootBalls(DcMotorEx leftLauncher, DcMotorEx rightLauncher, CRServo leftFeeder,
                           CRServo rightFeeder, DcMotor intake1, DcMotor intake2, double launchVelocity) {
        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        DcMotorEx.Direction LeftFeederDirection = leftFeeder.getDirection();
        DcMotorEx.Direction LeftLauncherDirection = leftLauncher.getDirection();
        DcMotorEx.Direction RightLauncherDirection = rightLauncher.getDirection();
        leftLauncher.setDirection(DcMotorEx.Direction.REVERSE);
        leftFeeder.setDirection(DcMotorEx.Direction.REVERSE);
        rightLauncher.setDirection(DcMotorEx.Direction.REVERSE);
        telemetry.addData("Left velocity", leftLauncher.getVelocity());
        telemetry.addData("Right Velocity", rightLauncher.getVelocity());
        telemetry.update();

        leftLauncher.setVelocity(launchVelocity);
        rightLauncher.setVelocity(-launchVelocity);
        for (byte i = 0; i <= 3; i++) {
            if (i != 0) {
                intake1.setPower(1);
                intake2.setPower(1);
            }
            rightFeeder.setPower(-1);
            leftFeeder.setPower(-1);
            sleep(1000);
            leftFeeder.setPower(0);
            rightFeeder.setPower(0);
            sleep(1000);
            telemetry.addData("Left velocity", leftLauncher.getVelocity());
            telemetry.addData("Right Velocity", rightLauncher.getVelocity());
            telemetry.update();
        }
        leftFeeder.setDirection(LeftFeederDirection);
        leftLauncher.setDirection(LeftLauncherDirection);
        rightLauncher.setDirection(RightLauncherDirection);
        leftLauncher.setPower(0);
        rightLauncher.setPower(0);
    }
}
