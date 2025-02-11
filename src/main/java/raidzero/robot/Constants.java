package raidzero.robot;

public class Constants {

    public class Limelight {

        public class Offsets {
            public static final double FL_X_OFFSET = -0.2155;
            public static final double FR_X_OFFSET = 0.2155;
            public static final double BL_X_OFFSET = -0.2445;
            public static final double BR_X_OFFSET = 0.2445;

            public static final double FL_Y_OFFSET = 0.2216;
            public static final double FR_Y_OFFSET = 0.2216;
            public static final double BL_Y_OFFSET = 0.2216;
            public static final double BR_Y_OFFSET = 0.2216;

            public static final double FL_Z_OFFSET = 0.2689;
            public static final double FR_Z_OFFSET = 0.2689;
            public static final double BL_Z_OFFSET = -0.2521;
            public static final double BR_Z_OFFSET = -0.2521;

            public static final double FL_ROLL = 0.0;
            public static final double FR_ROLL = 0.0;
            public static final double BL_ROLL = 0.0;
            public static final double BR_ROLL = 0.0;
            
            public static final double FL_PITCH = 20.0;
            public static final double FR_PITCH = 20.0;
            public static final double BL_PITCH = 35.0;
            public static final double BR_PITCH = 35.0;

            public static final double FL_YAW = -45.0;
            public static final double FR_YAW = 45.0;
            public static final double BL_YAW = 135.0;
            public static final double BR_YAW = -135.0;
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