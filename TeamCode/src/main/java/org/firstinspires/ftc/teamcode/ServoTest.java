package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp @SuppressWarnings("unused")
public class ServoTest extends M3_CommonFunctions {
    @Override
    public void runOpMode() {
        Servo arm = hardwareMap.get(Servo.class, "armServo");
        DcMotorEx launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        DcMotorEx intake1 = hardwareMap.get(DcMotorEx.class, "intake1");
        DcMotorEx intake2 = hardwareMap.get(DcMotorEx.class, "intake2");
        arm.scaleRange(0.5, 1);

        waitForStart();

        while (opModeIsActive()) {
            startLaunchers(launcher, 1400);
            if (gamepad1.a) {
                shootArtifacts(launcher, intake1, intake2, arm, 1400);
            } else {
                arm.setPosition(1);
            }
            intake1.setPower(1);
            intake2.setPower(-1);
            telemetry.update();
        }
    }
}
