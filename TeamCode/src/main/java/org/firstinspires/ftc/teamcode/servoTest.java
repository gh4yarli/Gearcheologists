package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;

@TeleOp
public class servoTest extends LinearOpMode{
    boolean counter = false;
    @Override
    public void runOpMode(){
        CRServo right_feeder = hardwareMap.get(CRServo.class, "right_feeder");
        CRServo left_feeder = hardwareMap.get(CRServo.class, "left_feeder");
        Servo servo = hardwareMap.get(Servo.class, "servo");
        DcMotor launcher = hardwareMap.get(DcMotor.class, "launcher");
        left_feeder.setDirection(DcMotorSimple.Direction.FORWARD);
        right_feeder.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Status","Inside Opmode");
            telemetry.update();
            left_feeder.setPower(1);
            right_feeder.setPower(1);
            servo.setPosition(0.5);
            servo.setPosition(0);
        }
    }
}
