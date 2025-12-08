package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
@TeleOp(name="ShootingTest")
public final class ShootingTest extends LinearOpMode {

    public static double launcherPower = 0.1;
    public static long delay = 1000;

    public static long servoTime = 790;

    public static double launchVelocity = 800;

    CRServo right_feeder;
    CRServo left_feeder;
    DcMotor intake1;
    DcMotor intake2;
    DcMotorEx launcher_left;
    DcMotorEx launcher_right;


    @Override
    public void runOpMode() {
        right_feeder = hardwareMap.get(CRServo.class, ConfigurationConstants.Names.RIGHT_FEEDER_SERVO);
        left_feeder = hardwareMap.get(CRServo.class, ConfigurationConstants.Names.LEFT_FEEDER_SERVO);
        intake1 = hardwareMap.get(DcMotorEx.class, ConfigurationConstants.Names.FIRST_INTAKE_MOTOR);
        intake2 = hardwareMap.get(DcMotorEx.class, ConfigurationConstants.Names.SECOND_INTAKE_MOTOR);
        launcher_left = hardwareMap.get(DcMotorEx.class, ConfigurationConstants.Names.LEFT_LAUNCHER_MOTOR);
        launcher_right = hardwareMap.get(DcMotorEx.class, ConfigurationConstants.Names.RIGHT_LAUNCHER_MOTOR);

        launcher_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        DcMotorEx.Direction LeftFeederDirection = left_feeder.getDirection();
        DcMotorEx.Direction LeftLauncherDirection = launcher_left.getDirection();
        DcMotorEx.Direction RightLauncherDirection = launcher_right.getDirection();
        launcher_left.setDirection(DcMotorEx.Direction.REVERSE);
        left_feeder.setDirection(DcMotorEx.Direction.REVERSE);
        launcher_right.setDirection(DcMotorEx.Direction.REVERSE);

        ElapsedTime timer = new ElapsedTime();
        //delay - 1000, vel - 790,   servo - 800

        waitForStart();
        if (opModeIsActive()) {
            // launcher_left.setPower(launcherPower);
            // launcher_right.setPower(launcherPower);
            launcher_left.setVelocity(launchVelocity);
            launcher_right.setVelocity(-launchVelocity);

            timer.reset();
            while(opModeIsActive() && timer.milliseconds() < 1500) {
                telemetry.addData("Left velocity", launcher_left.getVelocity());
                telemetry.addData("Right Velocity", launcher_right.getVelocity());
                telemetry.update();
                sleep(50);
            }


            //for (byte i = 0; i <= 3; i++) {
            while (opModeIsActive()) {
                if(!opModeIsActive()) break;

                // if (i != 0) {
                intake2.setPower(1);
                intake1.setPower(1);
                // }
                for (byte j = 0; j < 5; j++) {
                    right_feeder.setPower(-1);
                    left_feeder.setPower(-1);
                    //sleep(500);
                    //right_feeder.setPower(0);
                    //left_feeder.setPower(0);
                    //sleep(100);
                    //sleep(1000);
                }


                timer.reset();
                while(opModeIsActive() && timer.milliseconds() < servoTime) {
                    telemetry.addData("Left velocity", launcher_left.getVelocity());
                    telemetry.addData("Right Velocity", launcher_right.getVelocity());
                    telemetry.update();
                    sleep(50);
                }

                left_feeder.setPower(0);
                right_feeder.setPower(0);

                timer.reset();
                while(opModeIsActive() && timer.milliseconds() < delay) {
                    telemetry.addData("Left velocity", launcher_left.getVelocity());
                    telemetry.addData("Right Velocity", launcher_right.getVelocity());
                    telemetry.update();
                    sleep(50);
                }
            }
            left_feeder.setDirection(LeftFeederDirection);
            launcher_left.setDirection(LeftLauncherDirection);
            launcher_right.setDirection(RightLauncherDirection);
            launcher_left.setVelocity(0);
            launcher_right.setVelocity(0);
            intake1.setPower(0);
            intake2.setPower(0);
        }
    }
}
