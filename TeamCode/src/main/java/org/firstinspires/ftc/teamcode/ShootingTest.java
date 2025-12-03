package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Config
@TeleOp(name="ShootingTest")
public final class ShootingTest extends LinearOpMode {

    public static double launcherPower = 0.35;
    public static long delay = 750;

    public static long servoTime = 500;

    CRServo right_feeder;
    CRServo left_feeder;
    DcMotor intake1;
    DcMotor intake2;
    DcMotor launcher_left;
    DcMotor launcher_right;


    @Override
    public void runOpMode() {
        right_feeder = hardwareMap.get(CRServo.class, ConfigurationConstants.Names.RIGHT_FEEDER_SERVO);
        left_feeder = hardwareMap.get(CRServo.class, ConfigurationConstants.Names.LEFT_FEEDER_SERVO);
        intake1 = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FIRST_INTAKE_MOTOR);
        intake2 = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.SECOND_INTAKE_MOTOR);
        launcher_left = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.LEFT_LAUNCHER_MOTOR);
        launcher_right = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.RIGHT_LAUNCHER_MOTOR);


        DcMotorSimple.Direction LeftFeederDirection = left_feeder.getDirection();
        DcMotorSimple.Direction LeftLauncherDirection = launcher_left.getDirection();
        DcMotorSimple.Direction RightLauncherDirection = launcher_right.getDirection();
        launcher_left.setDirection(DcMotorSimple.Direction.REVERSE);
        left_feeder.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher_right.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
            launcher_left.setPower(launcherPower);
            launcher_right.setPower(launcherPower);
            sleep(2000);

            for (byte i = 0; i <= 4; i++) {
                if (i != 0) {
                    intake1.setPower(1);
                }
                right_feeder.setPower(-1);
                left_feeder.setPower(-1);
                intake2.setPower(1);
                sleep(servoTime);
                left_feeder.setPower(0);
                right_feeder.setPower(0);
                sleep(delay);
            }
            left_feeder.setDirection(LeftFeederDirection);
            launcher_left.setDirection(LeftLauncherDirection);
            launcher_right.setDirection(RightLauncherDirection);
            launcher_left.setPower(0);
            launcher_right.setPower(0);
            intake1.setPower(0);
            intake2.setPower(0);
        }
    }
}
