package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

        // Intakes and Launcher
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");

        // reverse the left motors just because that's how it works
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pid_right_new = new PIDFCoefficients(50,0.75,1.0,12.7);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid_right_new);

    }

    @Override
    public void loop(){


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


        intake1.setPower(-1.0);

        // this turns on both of the intakes 🎉
        if (gamepad2.left_trigger < -0.1){
            intake1.setPower(1.0);
            intake2.setPower(1.0);
        }
        // Reverse intakes in case the artifacts are jammed 😢
        else if (gamepad2.a) {
            intake2.setPower(1.0);
        }
        // if nothing is pressed nothing will happen 😱🤯
        else {
            intake1.setPower(0);
            intake2.setPower(0);
        }
        // if x is pressed it will launch the artifacts
        if (gamepad2.right_trigger > 0.1){

            intake2.setPower(-1.0);
            launcher.setVelocity(1400);
        }
        // if x is unpressed it will stop moving
         else {

             intake2.setPower(0);
             launcher.setPower(0);
        }

    /* YAY I NOW HAVE COMPLETED EXTREMELY EXTREMELY A LOT OF BASIC CODE 🥳🥳🥳🥳
        Now if the code works, add the telemtry 👍👍👍
        then make code for toggling field centric and slow mode🫡🫡🫡
        and then test 🙂🙂🙂🙂
        and the learn that it doesnt work 😡😤😡
        and then become depressed😢🙁💔😔
        and then get it 2 days before the due datedate 😎😎
        and then cook the competition 🤩👍
        (maybe)🤔💀💀💀💀
          */

    }



}