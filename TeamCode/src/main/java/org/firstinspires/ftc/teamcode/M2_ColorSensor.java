package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
public abstract class M2_ColorSensor extends LinearOpMode{
    public boolean detectArtifact(ColorSensor colorSensor){
        boolean isBallDetected = false;
        int blue = colorSensor.blue();
        int red = colorSensor.red();
        int green = colorSensor.green();
        int alpha = colorSensor.alpha();
        telemetry.addData("R", red);
        telemetry.addData("G", green);
        telemetry.addData("B", blue);
        telemetry.addData("A", alpha);
        return isBallDetected;
    }
}
