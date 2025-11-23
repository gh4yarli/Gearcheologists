package org.firstinspires.ftc.teamcode;

import android.media.Ringtone;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public abstract class M2_RedZone_IntakeAndLauncher extends LinearOpMode {
    /**
     * This method shoots 2 balls with a motor power of 40%
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
     */
    public void shootBalls(DcMotor leftLauncher, DcMotor rightLauncher, CRServo leftFeeder,
                           CRServo rightFeeder, DcMotor intake1, DcMotor intake2, double launcherPower)
    {
        DcMotorSimple.Direction LeftFeederDirection = leftFeeder.getDirection();
        DcMotorSimple.Direction LeftLauncherDirection = leftLauncher.getDirection();
        DcMotorSimple.Direction RightLauncherDirection = rightLauncher.getDirection();
        leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLauncher.setPower(launcherPower);
        rightLauncher.setPower(launcherPower);
        sleep(2000);
        for ( byte i = 0; i < 3; i++ ){
            if (i != 0 ){
                intake1.setPower(1);
                intake2.setPower(1);
            }
            rightFeeder.setPower(1);
            leftFeeder.setPower(1);
            sleep(750);
            leftFeeder.setPower(0);
            rightFeeder.setPower(0);
            sleep(1000);
        }
        leftFeeder.setDirection(LeftFeederDirection);
        leftLauncher.setDirection(LeftLauncherDirection);
        rightLauncher.setDirection(RightLauncherDirection);
    }

    /**
     * <p></p>
     * @param intake1 primary intake
     * @param intake2 secondary intake
     */
    public void intake(DcMotor intake1, DcMotor intake2){
        intake1.setPower(1);
        intake2.setPower(1);
        sleep(2500);
        intake2.setPower(0);
        sleep(2500);
        intake1.setPower(0);
    }
}