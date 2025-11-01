package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "Launch Test")
public final class LaunchTest extends LinearOpMode {
    public static double launchPower = 0.5;
    public static double intakePower = 0.5;
    public static double feedPower = 1;


    public DcMotorEx intake;
    public DcMotorEx launcher;
    public CRServo left_feeder;
    public CRServo right_feeder;


        @Override
        public void runOpMode() {
            launcher = hardwareMap.get(DcMotorEx.class, "launcher");
            intake = hardwareMap.get(DcMotorEx.class, "intake");
            left_feeder = hardwareMap.get(CRServo.class, "left_feeder");
            right_feeder = hardwareMap.get(CRServo.class, "right_feeder");
            ElapsedTime runtime = new ElapsedTime();

            waitForStart();

            while (opModeIsActive()) {
                if (gamepad1.left_bumper ) {
                    intake.setPower(intakePower);
                }
                if (gamepad1.right_bumper) {
                    left_feeder.setPower(-feedPower);
                    right_feeder.setPower(feedPower);
                    if (runtime.seconds() > 1) {
                        left_feeder.setPower(0);
                        right_feeder.setPower(0);
                        runtime.reset();
                    }
                }
                if (gamepad1.left_trigger > 0) {
                    launcher.setPower(launchPower);
                } else {
                    launcher.setPower(0);
                }
            }
    }
}
