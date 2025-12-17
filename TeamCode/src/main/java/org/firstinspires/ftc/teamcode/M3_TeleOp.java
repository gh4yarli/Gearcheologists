package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@TeleOp(name = "M3_CompTeleop", group = "Competition")
public class M3_TeleOp extends OpMode {

    // Mecanum Wheels
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    // Intake/Launcher
    DcMotorEx launcher;
    DcMotor intake1; // <-- CHANGE THE NAME TO THE CORRECT ONE
    DcMotor intake2;
    CRServo armServo;

    double forwardBackward;
    double strafeRightLeft;
    double rotate = 0;

    @Override
    public void init() {
        // initializing stuff
        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftDrive = hardwareMap.get(DcMotor.class, "leftBack");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightBack");

        // Intakes, Servo,  and Launcher
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        armServo = hardwareMap.get(CRServo.class,"armServo");

        // reverse the left motors just because that's how it works
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pid_right_new = new PIDFCoefficients(50,0.75,1.0,12.7);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid_right_new);

    }

    @Override
    public void loop() {


        // Driver Binds


        forwardBackward = -gamepad1.left_stick_y;
        strafeRightLeft = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;


        // Calculate power for each wheel

        double frontLeftPower = forwardBackward + strafeRightLeft + rotate;
        double frontRightPower = forwardBackward - strafeRightLeft - rotate;
        double backLeftPower = forwardBackward - strafeRightLeft + rotate;
        double backRightPower = forwardBackward + strafeRightLeft - rotate;

        // setting the maximum power value so IT doesn't mess it up
        //note: scaling it down proportionally in case value comes at to be more than 1
        double maxPower = 1.0;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        // the stuff below sends the power to the motor so it works! 🎉🎉🎉
        frontLeftDrive.setPower(frontLeftPower / maxPower);
        frontRightDrive.setPower(frontRightPower / maxPower);
        backLeftDrive.setPower(backLeftPower / maxPower);
        backRightDrive.setPower(backRightPower / maxPower);


        // INTAKE AND LAUNCHER CODE BELOW


        intake1.setPower(1.0);

        // this turns on both of the intake 2
        if (gamepad2.left_trigger > 0) {
            intake2.setPower(-1.0);
        //We need to add servo here

        }
        // if left bumper isn't pressed, intake 2 won't run
        else {
            intake2.setPower(0);
        }
        // if right trigger is pressed, it will launch the artifacts
        if (gamepad2.right_trigger > 0) {

            launcher.setVelocity(1380);

            if (gamepad2.right_bumper) {
                launcher.setVelocity(1800);
            }
        }
        // if x is unpressed it will stop moving
        else {

            intake2.setPower(0);
            launcher.setPower(0);

            if (gamepad2.b) {
                launcher.setVelocity(0);
                intake1.setPower(0);
                intake2.setPower(0);
                frontLeftDrive.setPower(0);
                frontRightDrive.setPower(0);
                backLeftDrive.setPower(0);
                backRightDrive.setPower(0);
            }

            }
        }


    }