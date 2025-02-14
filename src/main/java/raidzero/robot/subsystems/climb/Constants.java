package raidzero.robot.subsystems.climb;

import com.ctre.phoenix6.signals.GravityTypeValue;

public class Constants {
    public class Joint {
        public static final int MOTOR_ID = 15;

        public static final double CONVERSION_FACTOR = 1.0;

        public static final double KS = 0.0125;
        public static final double KG = 0.0275;
        public static final double KV = 1.1;
        public static final double KA = 0.05;

        public static final GravityTypeValue GRAVITY_TYPE_VALUE = GravityTypeValue.Arm_Cosine;

        public static final double KP = 0.0;
        public static final double KI = 0.0;
        public static final double KD = 0.0;

        public static final double CRUISE_VELOCITY = 0.3;
        public static final double ACCELERATION = 0.75;
        public static final double JERK = 0.0;

        // 2cm of tolerance
        public static final double POSITION_TOLERANCE_ROTATIONS = 0.02 * CONVERSION_FACTOR;

        public static final double STATOR_CURRENT_LIMT = 30.0;
        public static final double SUPPLY_CURRENT_LIMIT = 40.0;
        public static final double SUPPLY_CURRENT_LOWER_TIME = 0.0;
    }

    public class Winch {
        public static final int MOTOR_ID = 16;
    }
}
