package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.LED;
@Disabled

@TeleOp
public class ColorSense extends LinearOpMode {
    private ColorSensor colorTest;
    private int red;
    private int green;
    private int blue;
    private LED greenLED;
    private LED redLED;
    @Override
    public void runOpMode(){
        greenLED = hardwareMap.get(LED.class, "greenLED");
        redLED = hardwareMap.get(LED.class, "redLED");
        colorTest = hardwareMap.get(ColorSensor.class, "colorTest");
        telemetry.addData("Status","Ready");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()){
            int red = colorTest.red();
            int green = colorTest.green();
            int blue = colorTest.blue();
            colorTest.enableLed(true);
            if (red > 150 && red > green && red > blue) {
                greenLED.enableLight(false);
                redLED.enableLight(true);
                telemetry.addData("Status","Red");
                telemetry.update();
            } else if (green > 150 && green > red && green > blue) {
                redLED.enableLight(false);
                greenLED.enableLight(true);
                telemetry.addData("Status","Green");
                telemetry.update();
            } else {
                greenLED.enableLight(false);
                redLED.enableLight(false);
                telemetry.addData("Status","Not red or green");
                telemetry.update();
            }
        }
    }
}
