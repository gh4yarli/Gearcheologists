package org.firstinspires.ftc.teamcode.Meet1;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp
@Disabled

public class TouchSense extends LinearOpMode {
    private RevTouchSensor touchTest;
    private DcMotor motorTest;
    private DigitalChannel magnetTest;
    private boolean isTouched;

    @Override
    public void runOpMode() {
        touchTest = hardwareMap.get(RevTouchSensor.class, "touchTest");
        motorTest = hardwareMap.get(DcMotor.class, "motorTest");
        magnetTest = hardwareMap.get(DigitalChannel.class, "magnetTest");
        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            if (!magnetTest.getState()) {
                motorTest.setPower(0.5);
            } else if (magnetTest.getState()) {
                motorTest.setPower(-0.5);
            }
            if (touchTest.isPressed()){
                motorTest.setPower(0.5);
            } else {
                motorTest.setPower(-0.5);
            }
        }
    }
}
