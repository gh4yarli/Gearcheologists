package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class launchTest extends LinearOpMode {
    @Override
    public void runOpMode(){
        DcMotor rightLauncher = hardwareMap.get(DcMotor.class, "rightLauncher");
        DcMotor leftLauncher = hardwareMap.get(DcMotor.class, "leftLauncher");
        DcMotorEx intake1 = hardwareMap.get(DcMotorEx.class, "feeder");
        DcMotorEx intake2 = hardwareMap.get(DcMotorEx.class, "intake2");
        CRServo leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        CRServo rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()){
            rightLauncher.setPower(0.315);
            leftLauncher.setPower(0.315);
            sleep(2000);
            leftFeeder.setPower(-1);
            rightFeeder.setPower(1);
            sleep(1000);
            intake2.setPower(1);
            sleep(1000);
            intake1.setPower(1);
        }
        if (isStopRequested()){
            stop();
        }
   }
}
