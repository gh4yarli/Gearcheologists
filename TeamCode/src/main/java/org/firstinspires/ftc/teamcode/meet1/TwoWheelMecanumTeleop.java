package org.firstinspires.ftc.teamcode.meet1;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name = "TwoWheelMecanumTeleop", group = "Linear Opmode")
@Disabled

public class TwoWheelMecanumTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // TODO: Tune your drive characterization, see https://ftc-docs.firstinspires.org/en/latest/docs/software/road-runner/drive-constants-tuning.html
        // TODO: Set your inPerTick value, see https://ftc-docs.firstinspires.org/en/latest/docs/software/road-runner/odometry-tuning.html
        double inPerTick = 1.0; // Placeholder value

        // TODO: Get your IMU configured and make sure the name matches
        IMU imu = hardwareMap.get(IMU.class, "imu");

        TwoDeadWheelLocalizer localizer = new TwoDeadWheelLocalizer(hardwareMap, imu, inPerTick, new Pose2d(0, 0, 0));

        // TODO: make sure your config has motors with these names (or change them)
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        // TODO: reverse motor directions if needed
        // leftFront.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            // Update localizer
            localizer.update();

            // Get gamepad inputs
            PoseVelocity2d drivePowers = new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            );


            // Mecanum drive calculations
            double leftFrontPower = drivePowers.linearVel.x - drivePowers.linearVel.y - drivePowers.angVel;
            double leftBackPower = drivePowers.linearVel.x + drivePowers.linearVel.y - drivePowers.angVel;
            double rightFrontPower = drivePowers.linearVel.x + drivePowers.linearVel.y + drivePowers.angVel;
            double rightBackPower = drivePowers.linearVel.x - drivePowers.linearVel.y + drivePowers.angVel;


            // Set motor powers
            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightFront.setPower(rightFrontPower);
            rightBack.setPower(rightBackPower);

            // Telemetry
            telemetry.addData("X", localizer.getPose().position.x);
            telemetry.addData("Y", localizer.getPose().position.y);
            telemetry.addData("Heading (deg)", Math.toDegrees(localizer.getPose().heading.log()));
            telemetry.update();
        }
    }
}
