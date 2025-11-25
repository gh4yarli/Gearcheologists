package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public abstract class M2_Functions extends LinearOpMode {
    /**
     *
     * @param colorSensor color sensor to detect balls
     * @return isBallDetected
     * <p>
     * returns true if ball is detected, false if not
     */
    public boolean detectArtifact(ColorSensor colorSensor){
        boolean isBallDetected = false;
        int blue = colorSensor.blue();
        int red = colorSensor.red();
        int green = colorSensor.green();
        int alpha = colorSensor.alpha();
        telemetry.addData("R", red);
        telemetry.addData("G", green);
        telemetry.addData("B", blue);
        telemetry.addData("A", alpha);
        //return isBallDetected;
        return true;
    }
    /**
     * This method shoots 2 balls. It checks if the robot has a ball, and then it shoots if it detects a ball.
     * If there is no ball, if will not shoot
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
     * @param launcherPower
     * power to shoot balls
     * @param colorSensor
     * color sensor to detect balls
     */
    public void shootBalls(DcMotor leftLauncher, DcMotor rightLauncher, CRServo leftFeeder,
                           CRServo rightFeeder, DcMotor intake1, DcMotor intake2, double launcherPower, ColorSensor colorSensor) {
        DcMotorSimple.Direction LeftFeederDirection = leftFeeder.getDirection();
        DcMotorSimple.Direction LeftLauncherDirection = leftLauncher.getDirection();
        DcMotorSimple.Direction RightLauncherDirection = rightLauncher.getDirection();
        leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLauncher.setPower(launcherPower);
        rightLauncher.setPower(launcherPower);
        sleep(2000);
        if (detectArtifact(colorSensor)) {
            for (byte i = 0; i < 3; i++) {
                if (i != 0) {
                    intake1.setPower(1);
                    intake2.setPower(1);
                }
                rightFeeder.setPower(-1);
                leftFeeder.setPower(-1);
                sleep(750);
                leftFeeder.setPower(0);
                rightFeeder.setPower(0);
                sleep(1000);
            }
            leftFeeder.setDirection(LeftFeederDirection);
            leftLauncher.setDirection(LeftLauncherDirection);
            rightLauncher.setDirection(RightLauncherDirection);
        }
    }

    /**
     * <p></p>
     * @param intake1 primary intake
     * @param intake2 secondary intake
     */
    public void intake(DcMotor intake1, DcMotor intake2){
        intake1.setPower(1);
        intake2.setPower(1);
        //sleep(2500);
        intake1.setPower(0);
        //sleep(2500);
        intake2.setPower(0);
    }
}
