/*
Code for the following in TeleOp Mode
Controls:
1. Click right stick button in gamepad 2 to toggle between Robot Centric and Field Centric
2. Click B in Gamepad 1 as Kill Switch
3. Click right trigger in gamepad 1 to shoot balls
4. Click left trigger in gamepad 1 to turn on intake, release to turn off
5. Gamepad 2 arcade drive w/ both joysticks
Notes:
1. Robot Centric Drive and Field Centric Drive
2. Both Joysticks work in parallel (you can move while shooting)
 */
package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@TeleOp
public class ControllerDrive extends LinearOpMode {
    CRServo right_feeder, left_feeder;
    DcMotor launcher, front_left_drive, front_right_drive, back_left_drive, back_right_drive;
    double forward, strafe, turn;
    public void shootBalls(){
        launcher.setPower(-0.55);
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (timer.milliseconds() < 2000){
            sleep(1);
            forward = -gamepad2.left_stick_y;
            turn = gamepad2.right_stick_x;
            strafe = gamepad2.left_stick_x;

            front_left_drive.setPower((forward + strafe + turn) / 2);
            front_right_drive.setPower((forward - strafe - turn) / 2);
            back_left_drive.setPower((forward - strafe + turn) / 2);
            back_right_drive.setPower((forward + strafe - turn) / 2);
        }
        timer.reset();
        for (byte i = 0; i < 2; i++) {
            left_feeder.setPower(1);
            right_feeder.setPower(-1);
            while (timer.milliseconds() < 400){
                sleep(1);
                forward = -gamepad2.left_stick_y;
                turn = gamepad2.right_stick_x;
                strafe = gamepad2.left_stick_x;

                front_left_drive.setPower((forward + strafe + turn) / 2);
                front_right_drive.setPower((forward - strafe - turn) / 2);
                back_left_drive.setPower((forward - strafe + turn) / 2);
                back_right_drive.setPower((forward + strafe - turn) / 2);
            }
            timer.reset();
            left_feeder.setPower(0);
            right_feeder.setPower(0);
            while (timer.milliseconds() < 1000){
                sleep(1);
                forward = -gamepad2.left_stick_y;
                turn = gamepad2.right_stick_x;
                strafe = gamepad2.left_stick_x;

                front_left_drive.setPower((forward + strafe + turn) / 2);
                front_right_drive.setPower((forward - strafe - turn) / 2);
                back_left_drive.setPower((forward - strafe + turn) / 2);
                back_right_drive.setPower((forward + strafe - turn) / 2);
            }
            timer.reset();
        }
    }

    @Override
    public void runOpMode() {

        // ===== Initialize hardware =====
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        front_left_drive = hardwareMap.get(DcMotor.class, "left_front");
        front_right_drive = hardwareMap.get(DcMotor.class, "right_front");
        back_left_drive = hardwareMap.get(DcMotor.class, "left_back");
        back_right_drive = hardwareMap.get(DcMotor.class, "right_back");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");

        launcher = hardwareMap.get(DcMotor.class, "launcher");
        left_feeder = hardwareMap.get(CRServo.class, "left_feeder");
        right_feeder = hardwareMap.get(CRServo.class, "right_feeder");

        // ===== Safety check for missing hardware =====
        if (pinpoint == null) {
            telemetry.addLine("❌ ERROR: 'pinpoint' not found in configuration!");
            telemetry.update();
            sleep(5000);
            return;
        }

        if (front_left_drive == null || front_right_drive == null ||
                back_left_drive == null || back_right_drive == null) {
            telemetry.addLine("❌ ERROR: One or more drive motors not found!");
            telemetry.update();
            sleep(5000);
            return;
        }

        telemetry.addLine("✅ All hardware mapped successfully");
        telemetry.update();

        // ===== Motor setup =====
        back_left_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ===== Pinpoint setup =====
        pinpoint.setOffsets(83.0, -115.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        Pose2D startPose = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0);
        pinpoint.setPosition(startPose);

        boolean fieldCentric = false;
        boolean prevButton = false;
        ElapsedTime debounce = new ElapsedTime();

        waitForStart();

        // ===== Init loop =====
        while (opModeInInit()) {
            pinpoint.setHeading(0.0, AngleUnit.DEGREES);
            pinpoint.setPosition(startPose);
            pinpoint.update();
        }

        // ===== Main loop =====
        while (opModeIsActive()) {
            pinpoint.update();

            // Emergency stop, click b on gamepad 1
            if (gamepad1.b) break;

            // Shooting function
            if (gamepad1.right_trigger >= 0.85) {
                shootBalls();
            }

            // Toggle drive mode
            boolean currentButton = gamepad2.right_stick_button;
            if (currentButton && !prevButton && debounce.seconds() > 0.3) {
                fieldCentric = !fieldCentric;
                debounce.reset();
            }
            prevButton = currentButton;

            // Intake control
            if (gamepad1.left_trigger > 0.5) {
                intake.setPower(-0.5);
            } else {
                intake.setPower(0);
            }

            // Driving control
            if (fieldCentric) {
                moveRobotFieldCentricDrive(pinpoint, front_left_drive, front_right_drive, back_left_drive, back_right_drive);
                telemetry.addData("Mode", "🌍 Field Centric");
            } else {
                double forward = -gamepad2.left_stick_y;
                double turn = gamepad2.right_stick_x;
                double strafe = gamepad2.left_stick_x;

                front_left_drive.setPower((forward + strafe + turn) / 2);
                front_right_drive.setPower((forward - strafe - turn) / 2);
                back_left_drive.setPower((forward - strafe + turn) / 2);
                back_right_drive.setPower((forward + strafe - turn) / 2);

                telemetry.addData("Mode", "🤖 Robot Centric");
            } if (gamepad2.left_stick_x == 0 && gamepad2.left_stick_y == 0 && gamepad2.right_stick_x == 0){
                front_left_drive.setPower(0);
                front_right_drive.setPower(0);
                back_left_drive.setPower(0);
                back_right_drive.setPower(0);
            } if (gamepad1.right_trigger==0){
                launcher.setPower(0);
            }
            // Display position
            Pose2D pos = pinpoint.getPosition();
            telemetry.addData("Position", String.format(Locale.US,
                    "{X: %.1f, Y: %.1f, H: %.1f°}",
                    pos.getX(DistanceUnit.MM),
                    pos.getY(DistanceUnit.MM),
                    pos.getHeading(AngleUnit.DEGREES)));
            telemetry.update();
        }

        // ===== Stop all motors =====
        front_left_drive.setPower(0);
        front_right_drive.setPower(0);
        back_left_drive.setPower(0);
        back_right_drive.setPower(0);
        intake.setPower(0);
        launcher.setPower(0);
        left_feeder.setPower(0);
        right_feeder.setPower(0);
    }

    // ===== Helper: Field-Centric Drive =====
    private void moveRobotFieldCentricDrive(@NonNull GoBildaPinpointDriver pinpoint,
                                            DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br) {

        double heading = pinpoint.getHeading(AngleUnit.RADIANS);

        double forward = -gamepad2.left_stick_y;
        double strafe = gamepad2.left_stick_x;
        double rotate = gamepad2.right_stick_x;

        double rotX = strafe * Math.cos(heading) - forward * Math.sin(heading);
        double rotY = strafe * Math.sin(heading) + forward * Math.cos(heading);

        double pfl = rotY + rotX + rotate;
        double pfr = rotY - rotX - rotate;
        double pbl = rotY - rotX + rotate;
        double pbr = rotY + rotX - rotate;

        // Normalize to keep power within [-1, 1]
        double max = Math.max(1.0, Math.max(Math.abs(pfl), Math.max(Math.abs(pfr),
                Math.max(Math.abs(pbl), Math.abs(pbr)))));

        fl.setPower(pfl / max);
        fr.setPower(pfr / max);
        bl.setPower(pbl / max);
        br.setPower(pbr / max);
    }
}