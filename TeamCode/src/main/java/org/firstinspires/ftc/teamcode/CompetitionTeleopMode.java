/* Copyright (c) 2025 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/*
 * This OpMode illustrates how to program your robot to drive field relative.  This means
 * that the robot drives the direction you push the joystick regardless of the current orientation
 * of the robot.
 *
 * This OpMode assumes that you have four mecanum wheels each on its own motor named:
 *   front_left_motor, front_right_motor, back_left_motor, back_right_motor
 *
 *   and that the left motors are flipped such that when they turn clockwise the wheel moves backwards
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 */
@TeleOp(name = "Robot: TeleopMode", group = "Competition")

public class CompetitionTeleopMode extends OpMode {

    public static class Params {
        public final double parYTicks = PinpointLocalizer.PARAMS.parYTicks; ; // y position of the parallel encoder (in tick units)
        public final double perpXTicks = PinpointLocalizer.PARAMS.perpXTicks; // x position of the perpendicular encoder (in tick units)

        public double inPerTick = MecanumDrive.PARAMS.inPerTick;
    }

    public static Params PARAMS = new Params();

    public GoBildaPinpointDriver pinpoint;

    // This declares the four motors needed
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor intake;
    DcMotor launcher;
    CRServo left_feeder;
    CRServo right_feeder;
    final Double LAUNCHER_POWER = -.54 ;

    @Override
    public void init() {

        //pinpoint
        pinpoint = (hardwareMap.get(GoBildaPinpointDriver.class, ConfigurationConstants.Params.ODOMETRY_COMPUTER));

        //roadrunner configuration
        double mmPerTick = PARAMS.inPerTick * 25.4;
        pinpoint.setEncoderResolution(1 / mmPerTick, DistanceUnit.MM);
        pinpoint.setOffsets(mmPerTick * PARAMS.parYTicks, mmPerTick * PARAMS.perpXTicks, DistanceUnit.MM);

        //Encoder Directions
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        frontLeftDrive = hardwareMap.get(DcMotor.class, ConfigurationConstants.Params.FRONT_LEFT_DRIVE_MOTOR);
        frontRightDrive = hardwareMap.get(DcMotor.class, ConfigurationConstants.Params.FRONT_RIGHT_DRIVE_MOTOR);
        backLeftDrive = hardwareMap.get(DcMotor.class, ConfigurationConstants.Params.BACK_LEFT_DRIVE_MOTOR);
        backRightDrive = hardwareMap.get(DcMotor.class, ConfigurationConstants.Params.BACK_RIGHT_DRIVE_MOTOR);

        //setup intake motor
        intake = hardwareMap.get(DcMotor.class, ConfigurationConstants.Params.INTAKE_MOTOR);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        //Feeders Setup
        left_feeder = hardwareMap.get(CRServo.class, ConfigurationConstants.Params.LEFT_FEEDER_SERVO);
        right_feeder = hardwareMap.get(CRServo.class, ConfigurationConstants.Params.RIGHT_FEEDER_SERVO);

        //Launcher Setup
        launcher = hardwareMap.get(DcMotor.class, ConfigurationConstants.Params.LAUNCHER_MOTOR);

        // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
        // wires, you should remove these
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reset position/heading at init
        pinpoint.resetPosAndIMU();
    }

    @Override
    public void loop() {
        telemetry.addLine("Press A to reset Yaw");
        telemetry.addLine("Hold left bumper to drive in robot relative");
        telemetry.addLine("The left joystick sets the robot direction");
        telemetry.addLine("Moving the right joystick left and right turns the robot");

        pinpoint.update();

        telemetry.addData("double", gamepad1.right_trigger);
        telemetry.update();
        // If you press the A button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing
        if (gamepad1.a) {
            pinpoint.resetPosAndIMU();
        }
        // If you press the left bumper, you get a drive from the point of view of the robot
        // (much like driving an RC vehicle)
        if (gamepad1.left_bumper) {
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
        //TO-DO:  CHANGE TO GAMEPAD 2
        if (gamepad1.left_trigger > 0) {
            intake.setPower(-gamepad1.left_trigger);
        }
        else if (gamepad1.left_trigger == 0) {
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if (gamepad1.right_trigger > 0) {
            launcher.setPower(-gamepad1.right_trigger);
            right_feeder.setPower(-1);
            left_feeder.setPower(1);
        }
        else if (gamepad1.right_trigger == 0) {
            launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            right_feeder.setPower(0);
            left_feeder.setPower(0);
        }
        if (gamepad1.x){
            shootBallsVertex();
        }
    }

    // This routine drives the robot field relative
    private void driveFieldRelative(double forward, double right, double rotate) {

        telemetry.addLine("Hold left bumper to drive in robot relative");
        telemetry.addLine("The left joystick sets the robot direction");
        telemetry.addLine("Moving the right joystick left and right turns the robot");
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        double pinpoint_Heading = pinpoint.getHeading(AngleUnit.RADIANS);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                pinpoint_Heading);

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);

    }

    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }
    public void shootBallsVertex(){
        launcher.setPower(LAUNCHER_POWER);
        right_feeder.setPower(-1);
        left_feeder.setPower(1);
        intake.setPower(-.6);
    }
}