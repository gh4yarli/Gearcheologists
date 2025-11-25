package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class M2_FunctionsTest extends M2_Functions {
    @Override
    public void runOpMode(){
        DcMotor leftLauncher = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.LEFT_LAUNCHER_MOTOR);
        DcMotor rightLauncher = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.RIGHT_LAUNCHER_MOTOR);
        DcMotor firstIntake = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FIRST_INTAKE_MOTOR);
        DcMotor secondIntake = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.SECOND_INTAKE_MOTOR);
        CRServo leftFeeder = hardwareMap.get(CRServo.class, ConfigurationConstants.Names.LEFT_FEEDER_SERVO);
        CRServo rightFeeder = hardwareMap.get(CRServo.class, ConfigurationConstants.Names.RIGHT_FEEDER_SERVO);
        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, ConfigurationConstants.Names.COLOR_SENSOR);
        waitForStart();
        while (opModeIsActive()){
            //intake(firstIntake, secondIntake);
            //shootBalls(leftLauncher, rightLauncher, leftFeeder, rightFeeder, firstIntake, secondIntake, 0.35, colorSensor);
            detectArtifact(colorSensor);
            telemetry.update();
            //stop();
        }
        if (isStopRequested()){
            stop();
        }
    }
}
