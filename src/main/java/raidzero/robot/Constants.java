package raidzero.robot;

public class Constants {
    public class AlgaeIntake {
        public class Joint {
            public static final int MOTOR_ID = 0;
            public static final double CONVERSION_FACTOR = 1.0;

            public static final double INTAKE_POSITION_DEGREES = 0.0;
            public static final double HOME_POSITION_DEGREES = 0.0;

            public static final double KS = 0.0;
            public static final double KG = 0.0;
            public static final double KV = 0.0;
            public static final double KA = 0.0;

            public static final double KP = 0.0;
            public static final double KI = 0.0;
            public static final double KD = 0.0;

            public static final double CRUISE_VELOCITY = 0.0;
            public static final double ACCELERATION = 0.0;
            public static final double JERK = 0.0;

            // 2cm of tolerance
            public static final double POSITION_TOLERANCE_Degrees = 2.0 * CONVERSION_FACTOR;
        }
        public class Climb {
            public static final int MOTOR_ID = 0;
            public static final double CONVERSION_FACTOR = 1.0;

            public static final double INTAKE_POSITION_DEGREES = 0.0;
            public static final double HOME_POSITION_DEGREES = 0.0;

            public static final double KS = 0.0;
            public static final double KG = 0.0;
            public static final double KV = 0.0;
            public static final double KA = 0.0;

            public static final double KP = 0.0;
            public static final double KI = 0.0;
            public static final double KD = 0.0;

            public static final double CRUISE_VELOCITY = 0.0;
            public static final double ACCELERATION = 0.0;
            public static final double JERK = 0.0;

            // 2cm of tolerance
            public static final double POSITION_TOLERANCE_Degrees = 2.0 * CONVERSION_FACTOR;
        }

        public class Roller {
            public static final int MOTOR_ID = 0;
            public static final double CONVERSION_FACTOR = 45;

            public static final double ROLLER_SPEED = 0.2; // 20 %
        }
    }
}
