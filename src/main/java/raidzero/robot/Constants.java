package raidzero.robot;

public class Constants {

    public class Limelight {

        public class Offsets {
            public static final double LEFT_X_OFFSET = 0.0;
            public static final double RIGHT_X_OFFSET = 0.0;
            public static final double BACK_X_OFFSET = 0.0;

            public static final double LEFT_Y_OFFSET = 0.0;
            public static final double RIGHT_Y_OFFSET = 0.0;
            public static final double BACK_Y_OFFSET = 0.0;

            public static final double LEFT_Z_OFFSET = 0.0;
            public static final double RIGHT_Z_OFFSET = 0.0;
            public static final double BACK_Z_OFFSET = 0.0;

            public static final double LEFT_ROLL = 0.0;
            public static final double RIGHT_ROLL = 0.0;
            public static final double BACK_ROLL = 0.0;
            
            public static final double LEFT_PITCH = 0.0;
            public static final double RIGHT_PITCH = 0.0;
            public static final double BACK_PITCH = 0.0;

            public static final double LEFT_YAW = 0.0;
            public static final double RIGHT_YAW = 0.0;
            public static final double BACK_YAW = 0.0;
        }

    }

    public class Swerve {
        public static final double STICK_DEADBAND = 0.2;
    }

    public class TelescopingArm {
        public class Telescope {
            public static final int MOTOR_ID = 0;
            public static final double CONVERSION_FACTOR = 0.022 * Math.PI;

            public static final double MAX_LENGTH_M = 1.8288;
            public static final double MIN_LENGTH_M = 0.0; 

            public static final double KS = 0.0;
            public static final double KV = 0.0;
            public static final double KA = 0.0;
            public static final double KP = 0.0;
            public static final double KI = 0.0;
            public static final double KD = 0.0;

            public static final double CRUISE_VELOCITY = 0.0;
            public static final double ACCELERATION = 0.0;
            public static final double JERK = 0.0;

            // 2cm of tolerance
            public static final double POSITION_TOLERANCE_ROTATIONS = 0.02 * CONVERSION_FACTOR;
        }

        public class ArmJoint {
            public static final int MOTOR_ID = 1;
            public static final double CONVERSION_FACTOR = 1.0;
            public static final double LENGTH_M = 1.0 * CONVERSION_FACTOR;

            public static final double KS = 0.0;
            public static final double KV = 0.0;
            public static final double KA = 0.0;
            public static final double KP = 0.0;
            public static final double KI = 0.0;
            public static final double KD = 0.0;

            public static final double CRUISE_VELOCITY = 0.0;
            public static final double ACCELERATION = 0.0;
            public static final double JERK = 0.0;

            // 2cm of tolerance
            public static final double POSITION_TOLERANCE_ROTATIONS = 0.02 * CONVERSION_FACTOR;
        }

        public class Roller {
            public static final int MOTOR_ID = 0;

            public static final double ROLLER_SPEED = 0.2; // 2%
        }
    }
}