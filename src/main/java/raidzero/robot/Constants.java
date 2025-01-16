package raidzero.robot;

public class Constants {
    public class Swerve {
        public static final double STICK_DEADBAND = 0.2;
    }

    public class TelescopingArm {
        public class Telescope {
            public static final int MOTOR_ID = 10;
            public static final double PULLEY_RADIUS_M = 0.0243205;
            public static final double PLANETARY_GEAR_RATIO = 1 / 9;

            public static final double GROUND_OFFSET_M = 0.0; // TODO: Find this
            public static final double TIP_OFFSET_M = 0.0; // TODO: Find this

            public static final double PULLEY_TO_TIP_RATIO = 1 / 2;
            // * Mutliply by num rotations to get meters
            public static final double CONVERSION_FACTOR = PULLEY_RADIUS_M * (1 / (2 * Math.PI) * PLANETARY_GEAR_RATIO) *
                PULLEY_TO_TIP_RATIO;

            // TODO: find these
            public static final double MAX_LENGTH_M = 2.0 * CONVERSION_FACTOR;
            public static final double MIN_LENGTH_M = 0.0;

            // TODO: find these
            /*
             * To find ks: turn bot so gravity does not pull in direction of motor movement (probably on it's side), then slowly increase voltage until arm moves. That is the number
             * To find kg: point arm straight up, do same thing, subtract ks.
             * To find kv: point arm up, increase voltage until 1 rps. subtract kg and ks
             * to find ks: point arm up, increase voltage until 1 rps/s. subtract kg and ks.
             */
            public static final double KS = 0.0;
            public static final double KG = 0.0;
            public static final double KV = 0.0;
            public static final double KA = 0.0;

            public static final double KP = 0.0;
            public static final double KI = 0.0;
            public static final double KD = 0.0;

            // TODO: Find these
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

        // TODO: find these
        public static final double[] L4_SCORING_POS_M = { 0.0, 0.0 };
        public static final double[] L3_SCORING_POS_M = { 0.0, 0.0 };
        public static final double[] L2_SCORING_POS_M = { 0.0, 0.0 };
        public static final double[] L1_SCORING_POS_M = { 0.0, 0.0 };
        public static final double[] INTAKE_POS_M = { 0.0, 0.0 };
    }
}