package raidzero.robot.subsystems.telescopingarm;

import com.ctre.phoenix6.signals.GravityTypeValue;

public class Constants {
    public class Telescope {
        public static final int MOTOR_ID = 10;
        public static final double PULLEY_RADIUS_M = 0.0243205;
        public static final double PLANETARY_GEAR_RATIO = 9 / 1;

        public static final double PULLEY_TO_TIP_RATIO = 2;
        public static final double CONVERSION_FACTOR = 27.27; // PLANETARY_GEAR_RATIO * PULLEY_TO_TIP_RATIO;

        public static final double STATOR_CURRENT_LIMIT = 30.0;
        public static final double SUPPLY_CURRENT_LIMIT = 40.0;
        public static final double SUPPLY_CURRENT_LOWER_TIME = 0.0;

        public static final double FORWARD_SOFT_LIMIT_PERCENT = 1.0;

        public static final double KP = 35.0;
        public static final double KI = 0.0;
        public static final double KD = 0.0;

        public static final double KS = 0.045;
        public static final double KV = 0.3;
        public static final double KG = 0.06;
        public static final double KA = 0.01;

        public static final GravityTypeValue GRAVITY_TYPE_VALUE = GravityTypeValue.Elevator_Static;

        // TODO: create a slot 1 for going down

        public static final double CRUISE_VELOCITY = 2.0;
        public static final double ACCELERATION = 6.5;
        public static final double JERK = 0.0;

        // 2cm of tolerance
        public static final double POSITION_TOLERANCE_ROTATIONS = 0.02; // 2% of the full range of motion

        public static final double MAX_HEIGHT_M = 1.88;

        public static final double TOP_SOFT_LIMIT = 1.0; // 100% range of motion
        public static final double BOTTOM_SOFT_LIMIT = 0.0; // 0% range of motion
    }

    public class Joint {
        // * Zero degrees is pointing straight up!!!
        public static final int MOTOR_ID = 11;
        public static final int CANCODER_ID = 11;

        public static final double CANCODER_GEAR_RATIO = 1.0; //TODO: find this gear ratio
        public static final double CANCODER_OFFSET = -0.274170;

        private static final double PLANETARY_GEAR_RATIO = 9.0 / 1.0;
        private static final double PIVOT_GEAR_RATIO = 10.0 / 1.0;
        public static final double CONVERSION_FACTOR = PLANETARY_GEAR_RATIO * PIVOT_GEAR_RATIO;

        // * Set the magnet offset so that straight up is 90 degrees pi/2

        public static final double KS = 0.0125;
        public static final double KG = 0.0275;
        public static final double KV = 1.1;
        public static final double KA = 0.05;

        public static final GravityTypeValue GRAVITY_TYPE_VALUE = GravityTypeValue.Arm_Cosine;

        public static final double KP = 100.0;
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

    public class Roller {
        public static final int MOTOR_ID = 12;
        public static final int FOLLOW_ID = 13;

        public static final double ROLLER_SPEED = 0.35; // 35%

        public static final double CURRENT_LIMIT = 25.0; // subject to change
    }

    // TODO: find these
    public static final double[] L4_SCORING_POS_M = { 0.0, 0.0 };
    public static final double[] L3_SCORING_POS_M = { -0.1, 0.8 };
    public static final double[] L2_SCORING_POS_M = { -0.1, 0.1 };
    public static final double[] L1_SCORING_POS_M = { 0.0, 0.0 };

    public static final double[] INTAKE_POS_M = { 0.0, 0.0 };

    public static final double[] HOME_POS_M = { 0.0, 0.0 };
}