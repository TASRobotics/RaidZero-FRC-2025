package raidzero.robot.subsystems.algaeintake;

public class Constants {
    public class Joint {
        public static final int MOTOR_ID = 12;
        public static final double CONVERSION_FACTOR = 9.0;

        public static final double INTAKE_POSITION = 0.0;
        public static final double HOME_POSITION = 0.0;

        public static final double KS = 0.015;
        public static final double KG = 0.0375;
        public static final double KV = 0.0;
        public static final double KA = 0.0;

        public static final double KP = 10.0;
        public static final double KI = 0.0;
        public static final double KD = 0.0;

        public static final double CURRENT_LIMIT = 5.0;

        public static final double POSITION_TOLERANCE = 1.0 / 360.0;

        public static final double FORWARD_SOFT_LIMIT = 90.0 / 360.0;
        public static final double REVERSE_SOFT_LIMIT = -135.0 / 360;

        public static final double CURRENT_SPIKE_THRESHOLD_AMPS = 25.0;
    }

    public class Roller {
        public static final int MOTOR_ID = 0;
        public static final double CONVERSION_FACTOR = 45;

        public static final double ROLLER_SPEED = 0.2; // 20 %
    }
}
