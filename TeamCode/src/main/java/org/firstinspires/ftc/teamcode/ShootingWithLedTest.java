package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LED;

@TeleOp
@SuppressWarnings({"unused", "FieldCanBeLocal"})
public class ShootingWithLedTest extends M3_CommonFunctions {
    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;
    private LED rightRedLed;
    private LED leftRedLed;
    private LED leftGreenLed;
    private LED rightGreenLed;

    @Override
    public void runOpMode() {
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightLauncher");
        rightRedLed = hardwareMap.get(LED.class, "rightLEDred");
        leftRedLed = hardwareMap.get(LED.class, "leftLEDred");
        leftGreenLed = hardwareMap.get(LED.class, "leftLEDgreen");
        rightGreenLed = hardwareMap.get(LED.class, "rightLEDgreen");
        CRServo leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        CRServo rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        boolean leftAtDesiredSpeed;
        boolean rightAtDesiredSpeed;

        int launcherVel = 850;
        double rightLauncherPower = 0.35;
        double leftLauncherPower = 0.35;
        double velRange = 100;
        double minVel = launcherVel - velRange;
        double maxVel = launcherVel + velRange;
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {
            //startLaunchers(leftMotor, rightMotor, maxVel);
            leftMotor.setPower(leftLauncherPower);
            rightMotor.setPower(-rightLauncherPower);
            leftAtDesiredSpeed = leftMotor.getVelocity() >= minVel && leftMotor.getVelocity() <= maxVel;
            rightAtDesiredSpeed = Math.abs(rightMotor.getVelocity()) >= minVel && Math.abs(rightMotor.getVelocity()) <= maxVel;
            if (gamepad1.a){
                startFeeder(leftFeeder, rightFeeder);
            } else {
                stopFeeder(leftFeeder, rightFeeder);
            }
            if (leftAtDesiredSpeed && rightAtDesiredSpeed) {
                leftGreenLed.enableLight(true);
                rightGreenLed.enableLight(true);
                leftRedLed.enableLight(false);
                rightRedLed.enableLight(false);
                startFeeder(leftFeeder, rightFeeder);
            } else {
                leftRedLed.enableLight(true);
                rightRedLed.enableLight(true);
                leftGreenLed.enableLight(false);
                rightGreenLed.enableLight(false);
                stopFeeder(leftFeeder, rightFeeder);
            }
            if (Math.abs(rightMotor.getVelocity()) < minVel){
                rightLauncherPower += 0.01;
            } if (Math.abs(rightMotor.getVelocity()) > maxVel){
                rightLauncherPower -= 0.01;
            } if (Math.abs(leftMotor.getVelocity()) < minVel){
                leftLauncherPower += 0.01;
            } if (Math.abs(leftMotor.getVelocity()) > maxVel){
                leftLauncherPower -= 0.01;
            }

            telemetry.addData("leftMotor", "Velocity: " + leftMotor.getVelocity());
            telemetry.addData("rightMotor", "Velocity: " + rightMotor.getVelocity());
            telemetry.update();
        }
    }
}