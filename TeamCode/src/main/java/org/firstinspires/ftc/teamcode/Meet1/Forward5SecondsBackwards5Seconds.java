package org.firstinspires.ftc.teamcode.Meet1;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.concurrent.TimeUnit;

@TeleOp
@Disabled

public class Forward5SecondsBackwards5Seconds extends LinearOpMode {
    private DcMotor motorTest;
    private Servo servoTest;

    private LED greenLED;

    @Override
    public void runOpMode() {
        motorTest = hardwareMap.get(DcMotor.class, "motorTest");
        servoTest = hardwareMap.get(Servo.class, "servoTest");
        greenLED = hardwareMap.get(LED.class, "greenLED");

        greenLED.enableLight(false);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
            motorTest.setPower(0.5);
            try {
                TimeUnit.SECONDS.sleep(5);
            } catch (InterruptedException e) {
                //to make the block work
            }
            motorTest.setPower(0);
            try {
                TimeUnit.SECONDS.sleep(1);
            } catch (InterruptedException e) {
                //to make the block work
            }
            motorTest.setPower(-0.5);
            try {
                TimeUnit.SECONDS.sleep(5);
            } catch (InterruptedException e) {
                //to make the block work
            }
            motorTest.setPower(0);
            try {
                TimeUnit.SECONDS.sleep(1);
            } catch (InterruptedException e) {
                //to make the block work
            }

        }
    }
}
