package org.firstinspires.ftc.teamcode;

/*
use this file to get the correct names for your hardwaremap like this:
DcMotor front_left_drive = hardwaremap.get(DcMotor.class, ConfigurationConstants.Names.FRONT_LEFT_DRIVE_MOTOR);
the above example line will get the correct configuration name for the front left drive motor.
*/

public final class ConfigurationConstants {
    public static class Names {
        public static final String FRONT_LEFT_DRIVE_MOTOR = "leftFront";
        public static final String FRONT_RIGHT_DRIVE_MOTOR = "rightFront";
        public static final String BACK_LEFT_DRIVE_MOTOR = "leftBack";
        public static final String BACK_RIGHT_DRIVE_MOTOR = "rightBack";
        public static final String INTAKE_MOTOR = "intake";
        public static final String LAUNCHER_MOTOR = "launcher";

        public static final String LEFT_FEEDER_SERVO = "left_feeder";
        public static final String RIGHT_FEEDER_SERVO = "right_feeder";

        public static final String DISTANCE_SENSOR = "distance";

        public static final String ODOMETRY_COMPUTER = "pinpoint";
    }
}
