package org.firstinspires.ftc.teamcode.meet2;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.ConfigurationConstants;

@TeleOp
public class M2_DistanceSensor extends M2_RedZone_IntakeAndLauncher {
    @Override
    public void runOpMode(){
        DcMotor leftLauncher = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.LEFT_LAUNCHER_MOTOR);
        DcMotor rightLauncher = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.RIGHT_LAUNCHER_MOTOR);
        DcMotor firstIntake = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FIRST_INTAKE_MOTOR);
        DcMotor secondIntake = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.SECOND_INTAKE_MOTOR);
        CRServo leftFeeder = hardwareMap.get(CRServo.class, ConfigurationConstants.Names.LEFT_FEEDER_SERVO);
        CRServo rightFeeder = hardwareMap.get(CRServo.class, ConfigurationConstants.Names.RIGHT_FEEDER_SERVO);
        waitForStart();
        if (opModeIsActive()){
            intake(firstIntake, secondIntake);
            shootBalls(leftLauncher, rightLauncher, leftFeeder, rightFeeder, firstIntake, secondIntake, 0.4);
            stop();
        }
        if (isStopRequested()){
            stop();
        }
    }
}
