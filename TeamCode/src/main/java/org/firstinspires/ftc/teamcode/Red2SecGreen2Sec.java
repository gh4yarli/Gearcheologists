package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class Red2SecGreen2Sec extends LinearOpMode {

    private LED greenLED;
    private LED redLED;
    private DistanceSensor distanceTest;

    @Override
    public void runOpMode() {
        greenLED = hardwareMap.get(LED.class, "greenLED");
        redLED = hardwareMap.get(LED.class, "redLED");
        distanceTest = hardwareMap.get(DistanceSensor.class,"distanceTest");
        waitForStart();
        while (opModeIsActive()) {

            if (distanceTest.getDistance(DistanceUnit.CM) >= 35) {
                redLED.enableLight(true);
                greenLED.enableLight(false);
                telemetry.addData("Color", "Green");
                telemetry.update();
            } else if (distanceTest.getDistance(DistanceUnit.CM) < 35 && distanceTest.getDistance(DistanceUnit.CM) > 25) {
                greenLED.enableLight(true);
                redLED.enableLight(true);
                telemetry.addData("Color", "Yellow");
                telemetry.update();
            } else if (distanceTest.getDistance(DistanceUnit.CM) <= 25) {
                greenLED.enableLight(true);
                redLED.enableLight(false);
                telemetry.addData("Color", "Red");
                telemetry.update();
            }

        }
    }
}
