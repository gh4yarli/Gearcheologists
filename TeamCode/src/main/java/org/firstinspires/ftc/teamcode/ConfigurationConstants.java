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
         * name for front left drive motor
         */
        public static final String FRONT_LEFT_DRIVE_MOTOR = "leftFront";
        /**
         * name for front right drive motor
         */
        public static final String FRONT_RIGHT_DRIVE_MOTOR = "rightFront";
        /**
         * name for back left drive motor
         */
        public static final String BACK_LEFT_DRIVE_MOTOR = "leftBack";
        /**
         * name for back right drive motor
         */
        public static final String BACK_RIGHT_DRIVE_MOTOR = "rightBack";

        //public static final String FIRST_INTAKE_MOTOR = "feeder";
        // For Meet 3
        public static final String FIRST_INTAKE_MOTOR = "intake1";
        public static final String SECOND_INTAKE_MOTOR = "intake2";

        public static final String LEFT_LAUNCHER_MOTOR = "leftLauncher";
        public static final String RIGHT_LAUNCHER_MOTOR = "rightLauncher";

        //For Meet3
        public static final String LAUNCHER_MOTOR = "launcher";

        public static final String LEFT_FEEDER_SERVO = "leftFeeder";
        public static final String RIGHT_FEEDER_SERVO = "rightFeeder";
        public static final String ARM_SERVO = "armServo";

        public static final String ODOMETRY_COMPUTER = "pinpoint";
    }
    public static final double SMALL_TRI_SHOOTING_TIME = 3.8;
    public static final double BIG_TRI_SHOOTING_TIME = 2.9;
}
