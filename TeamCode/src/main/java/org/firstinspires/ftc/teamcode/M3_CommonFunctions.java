package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;


public abstract class M3_CommonFunctions extends LinearOpMode {

    /**
     * Starts the launchers
     * @param leftLauncher
     * left launcher motor
     * @param rightLauncher
     * right launcher motor
     * @param launcherVel
     * power to shoot balls
     */
    public void startLaunchers(DcMotorEx leftLauncher, DcMotorEx rightLauncher, double launcherVel) {
        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLauncher.setVelocity(launcherVel);
        rightLauncher.setVelocity(-launcherVel);
    }

    /**
     * Stops the launchers
     * @param leftLauncher
     * left launcher motor
     * @param rightLauncher
     * right launcher motor
     */
    public void stopLaunchers(DcMotorEx leftLauncher, DcMotorEx rightLauncher) {
        // setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE) tries to stop the DcMotor instead of just letting it spin
        // when a power of 0 is requested
        leftLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLauncher.setVelocity(0);
        rightLauncher.setVelocity(0);
    }

    /**
     * Starts the feeders
     * @param leftFeeder
     * left feeder servo
     * @param rightFeeder
     * right feeder servo
     */
    public void startFeeder(CRServo leftFeeder, CRServo rightFeeder) {
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setPower(-1);
        rightFeeder.setPower(-1);
    }

    /**
     * Stops the feeders
     * @param leftFeeder
     * left feeder servo
     * @param rightFeeder
     * right feeder servo
     */
    public void stopFeeder(CRServo leftFeeder, CRServo rightFeeder) {
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
    }

    /**
     * Starts the intake
     * @param intake1
     * first intake
     * @param intake2
     * second intake
     */
    public void startIntake(DcMotor intake1, DcMotor intake2) {
        intake1.setPower(1);
        intake2.setPower(-1);
    }

    /**
     * Stops the intake
     * @param intake1
     * first intake
     * @param intake2
     * second intake
     */
    public void stopIntake(DcMotor intake1, DcMotor intake2) {
        intake1.setPower(0);
        intake2.setPower(0);
    }

    /**
     * <p></p>
     * function to shoot balls
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
    public void shootBalls(DcMotorEx leftLauncher, DcMotorEx rightLauncher, CRServo leftFeeder,
                           CRServo rightFeeder, DcMotor intake1, DcMotor intake2) {
        DcMotorEx.Direction LeftFeederDirection = leftFeeder.getDirection();
        DcMotorEx.Direction LeftLauncherDirection = leftLauncher.getDirection();
        DcMotorEx.Direction RightLauncherDirection = rightLauncher.getDirection();
        sleep(2000);
        for (byte i = 0; i < 4; i++) {
            if (i != 0) {
                startIntake(intake1, intake2);
            }
            startFeeder(leftFeeder, rightFeeder);
            sleep(1000);
            stopFeeder(leftFeeder, rightFeeder);
            sleep(1000);
        }
        stopLaunchers(leftLauncher, rightLauncher);
        stopIntake(intake1, intake2);
        leftFeeder.setDirection(LeftFeederDirection);
        leftLauncher.setDirection(LeftLauncherDirection);
        rightLauncher.setDirection(RightLauncherDirection);
    }
    public void shootArtifacts(DcMotorEx launcher, DcMotor intake1, DcMotor intake2, double launcherVel) {

        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pid_right_new = new PIDFCoefficients(50,0.75,1.0,12.7);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid_right_new);
        ElapsedTime runtime = new ElapsedTime();
        while (runtime.seconds() < 4) {
            launcher.setVelocity(-launcherVel);

            boolean launcherAtSpeed = Math.abs(launcher.getVelocity()) >= launcherVel - 50 && Math.abs(launcher.getVelocity()) <= launcherVel + 50;

            if (launcherAtSpeed) {
                startIntake(intake1, intake2);
            }
            telemetry.addData("motor velocity", Math.abs(launcher.getVelocity()));
            telemetry.update();
        }
        launcher.setVelocity(0);
        intake2.setPower(0);
    }

}
