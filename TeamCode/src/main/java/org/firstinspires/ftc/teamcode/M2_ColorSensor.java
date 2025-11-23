package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
public class M2_ColorSensor {
    public boolean detectArtifact(ColorSensor colorSensor){
        boolean isBallDetected = false;
        int torf = (int) Math.round(Math.random() * 1000.0);
        if (torf % 2 == 0){
            isBallDetected = true;
        }
        return isBallDetected;
    }
}
