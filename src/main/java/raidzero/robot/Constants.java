package raidzero.robot;

public class Constants {
    public class Swerve {
        public static final double STICK_DEADBAND = 0.2;
    }

    public class TelescopingArm {
        public static final int TELESCOPE_MOTOR_ID = 0;
        public static final int ARM_MOTOR_ID = 1;
        public static final int WRIST_MOTOR_ID = 2;

        public static final double TELESCOPE_CONVERSION_FACTOR = 1.0;
        public static final double ARM_CONVERSION_FACTOR = 1.0;
        public static final double WRIST_CONVERSION_FACTOR = 1.0;

        public static final double TELESCOPE_MAX_LENGTH_M = 1.0 * TELESCOPE_CONVERSION_FACTOR;
        public static final double ARM_LENGTH_M = 1.0 * TELESCOPE_CONVERSION_FACTOR;

        // TELESCOPE
        public static final double TELESCOPE_KS = 0.0;
        public static final double TELESCOPE_KV = 0.0;
        public static final double TELESCOPE_KA = 0.0;
        public static final double TELESCOPE_KP = 0.0;
        public static final double TELESCOPE_KI = 0.0;
        public static final double TELESCOPE_KD = 0.0;

        public static final double TELESCOPE_CRUISE_VELOCITY = 0.0;
        public static final double TELESCOPE_ACCELERATION = 0.0;
        public static final double TELESCOPE_JERK = 0.0;

        public static final double TELESCOPE_POSITION_TOLERANCE_M = 0.02;

        // ARM
        public static final double ARM_KS = 0.0;
        public static final double ARM_KV = 0.0;
        public static final double ARM_KA = 0.0;
        public static final double ARM_KP = 0.0;
        public static final double ARM_KI = 0.0;
        public static final double ARM_KD = 0.0;

        public static final double ARM_CRUISE_VELOCITY = 0.0;
        public static final double ARM_ACCELERATION = 0.0;
        public static final double ARM_JERK = 0.0;

        public static final double ARM_POSITION_TOLERANCE_M = 0.02;

        // WRIST
        public static final double WRIST_KS = 0.0;
        public static final double WRIST_KV = 0.0;
        public static final double WRIST_KA = 0.0;
        public static final double WRIST_KP = 0.0;
        public static final double WRIST_KI = 0.0;
        public static final double WRIST_KD = 0.0;

        public static final double WRIST_CRUISE_VELOCITY = 0.0;
        public static final double WRIST_ACCELERATION = 0.0;
        public static final double WRIST_JERK = 0.0;

        public static final double WRIST_POSITION_TOLERANCE_M = 0.02;
    }
}