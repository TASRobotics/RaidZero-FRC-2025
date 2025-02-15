package raidzero.robot.subsystems.algaeintake;

import com.ctre.phoenix6.signals.GravityTypeValue;

public class Constants {
    public class JointC {
        public static final int MOTOR_ID = 12;
        public static final double CONVERSION_FACTOR = (54 / 30) * 9;

        public static final double INTAKE_POSITION = 0.0;
        public static final double HOME_POSITION = 0.0;

        public static final double KS = 0.025390625;
        public static final double KG = 0.02734375;
        public static final double KV = 0.5555555820465088;
        public static final double KA = 0.014000000432133675;

        public static final double KP = 20.0;
        public static final double KI = 0.0;
        public static final double KD = 0.0;

        public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;

        public static final double CURRENT_LIMIT = 30.0;
        public static final double SUPPLY_CURRENT_LIMIT = 40.0;
        public static final double SUPPLY_CURRENT_LOWER_TIME = 0.0;

        public static final double MOTION_MAGIC_CRUISE_VELOCITY = 0.7;
        public static final double MOTION_MAGIC_ACCELERATION = 4.5;

        public static final double CURRENT_SPIKE_THRESHOLD_AMPS = 25.0;
    }

    public class Roller {
        public static final int MOTOR_ID = 0;
        public static final double CONVERSION_FACTOR = 45;

        public static final double ROLLER_SPEED = 0.2; // 20 %
    }
}
