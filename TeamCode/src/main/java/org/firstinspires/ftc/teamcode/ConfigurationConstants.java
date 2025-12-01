package org.firstinspires.ftc.teamcode;

public final class ConfigurationConstants {
    /**
     * Class that stores all configuration names. Use it by calling the class like this:
     * <p></p>
     * DcMotor exampleMotor = hardwaremap.get(DcMotor.class, ConfigurationConstants.Names.EXAMPLE_MOTOR);
     *
     */
    public static class Names {
        /**
         * name for the front left drive motor
         */
        public static final String FRONT_LEFT_DRIVE_MOTOR = "leftFront";
        /**
         * name for the front right drive motor
         */
        public static final String FRONT_RIGHT_DRIVE_MOTOR = "rightFront";
        /**
         * name for the back left drive motor
         */
        public static final String BACK_LEFT_DRIVE_MOTOR = "leftBack";
        /**
         * name for the back right drive motor
         */
        public static final String BACK_RIGHT_DRIVE_MOTOR = "rightBack";

        public static final String FIRST_INTAKE_MOTOR = "feeder";
        public static final String SECOND_INTAKE_MOTOR = "intake2";
        public static final String LEFT_LAUNCHER_MOTOR = "leftLauncher";
        public static final String RIGHT_LAUNCHER_MOTOR = "rightLauncher";

        public static final String LEFT_FEEDER_SERVO = "leftFeeder";
        public static final String RIGHT_FEEDER_SERVO = "rightFeeder";

        public static final String BACK_DISTANCE_SENSOR = "backDistance";
        public static final String FRONT_DISTANCE_SENSOR = "frontDistance";
        public static final String COLOR_SENSOR = "color";

        public static final String ODOMETRY_COMPUTER = "pinpoint";
    }
}