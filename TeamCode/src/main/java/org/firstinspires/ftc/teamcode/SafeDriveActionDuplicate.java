package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public abstract class SafeDriveActionDuplicate implements Action {
    private Action baseAction;
    private Rev2mDistanceSensor sensor;
    private double obstacleThreshold;
    private boolean isPaused = false;

    public MecanumDrive drive;

    public SafeDriveActionDuplicate(Action baseAction, Rev2mDistanceSensor sensor, double obstacleThreshold, MecanumDrive drive) {
        this.baseAction = baseAction;
        this.sensor = sensor;
        this.obstacleThreshold = obstacleThreshold;
    }

    public boolean run(@NonNull TelemetryPacket packet, MecanumDrive drive) {
        // Check sensor
        double distance = sensor.getDistance(DistanceUnit.CM);
        boolean validReading = (distance >= 5 && distance <= 200);

        if (validReading && distance < obstacleThreshold) {
            // Obstacle detected - don't run base action
            isPaused = true;
            packet.put("Status", "PAUSED - Obstacle at " + String.format("%.1f", distance) + "cm");
            drive.leftFront.setPower(0);
            drive.leftBack.setPower(0);
            drive.rightBack.setPower(0);
            drive.rightFront.setPower(0);
            return true; // Still running, just paused
        }

        // Path is clear - run base action
        if (isPaused) {
            packet.put("Status", "Resuming");
            isPaused = false;
        }

        return baseAction.run(packet);
    }
}

// Usage:
// Action safePath = new SafeDriveAction(normalPath, frontSensor, 40.0);
// Actions.runBlocking(safePath);
