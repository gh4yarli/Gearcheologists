package org.firstinspires.ftc.teamcode.Meet1;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Disabled

@TeleOp
public class DistanceLEDMotor extends LinearOpMode {
    private LED greenLED;
    private LED redLED;
    private DistanceSensor distanceTest;
    private DcMotor motorTest;

    @Override
    public void runOpMode() {
        greenLED = hardwareMap.get(LED.class, "greenLED");
        redLED = hardwareMap.get(LED.class, "redLED");
        distanceTest = hardwareMap.get(DistanceSensor.class, "distanceTest");
        motorTest = hardwareMap.get(DcMotor.class, "motorTest");
        telemetry.addData("Status","Ready");
        telemetry.update();
        waitForStart();
        
        while (opModeIsActive()){
            if (distanceTest.getDistance(DistanceUnit.CM) > 20) {
                double power = distanceTest.getDistance(DistanceUnit.CM)/180;
                if (power > 0.5){
                    power = 0.5;
                } else if (power < 0.2) {
                    power = 0.2;
                }
                motorTest.setPower(power);
                greenLED.enableLight(true);
                redLED.enableLight(false);
                telemetry.addData("Status", "Moving forward.");
                telemetry.update();
            } else if (distanceTest.getDistance(DistanceUnit.CM) <= 20 && distanceTest.getDistance(DistanceUnit.CM) >= 10) {
                motorTest.setPower(0.1);
                telemetry.addData("Status", "Obstacle detected. Slowing down.");
                telemetry.update();
            } else if (distanceTest.getDistance(DistanceUnit.CM) < 10) {
                double power = 1/distanceTest.getDistance(DistanceUnit.CM);
                if (power > 0.5) {
                    power = 0.5;
                } else if (power < 0.2) {
                    power = 0.2;
                }
                motorTest.setPower(power*(-1));
                redLED.enableLight(true);
                greenLED.enableLight(false);
                telemetry.addData("Status", "Obstacle is too close. Moving backwards");
                telemetry.update();
            }
        }
    }
}