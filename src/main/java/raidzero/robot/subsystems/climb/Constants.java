package raidzero.robot.subsystems.climb;

import com.ctre.phoenix6.signals.GravityTypeValue;

public class Constants {
    public class Joint {
        public static final int MOTOR_ID = 15;

        public static final double CONVERSION_FACTOR = 80.0 / 10.0;

        public static final double KS = 0.0;
        public static final double KG = 0.0;
        public static final double KV = 0.0;
        public static final double KA = 0.0;

        public static final GravityTypeValue GRAVITY_TYPE_VALUE = GravityTypeValue.Arm_Cosine;

        public static final double KP = 0.0;
        public static final double KI = 0.0;
        public static final double KD = 0.0;

        public static final double CRUISE_VELOCITY = 0.0;
        public static final double ACCELERATION = 0.0;
        public static final double JERK = 0.0;

        // 2cm of tolerance
        public static final double POSITION_TOLERANCE_ROTATIONS = 0.02 * CONVERSION_FACTOR;

        public static final double STATOR_CURRENT_LIMT = 20.0;
        public static final double SUPPLY_CURRENT_LIMIT = 20.0;
        public static final double SUPPLY_CURRENT_LOWER_TIME = 0.0;
    }

    public class Winch {
        public static final int MOTOR_ID = 16;
    }
}
