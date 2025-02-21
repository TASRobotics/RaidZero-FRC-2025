package raidzero.robot;

import com.ctre.phoenix6.signals.GravityTypeValue;

public class Constants {

    public class Swerve {
        public static final double STICK_DEADBAND = 0.2;
    }

    public class TelescopingArm {
        public class Telescope {
            public static final int MOTOR_ID = 10;

            // TODO: find new conversion factor
            public static final double CONVERSION_FACTOR = 52.643555;

            public static final double STATOR_CURRENT_LIMIT = 40.0;
            public static final double SUPPLY_CURRENT_LIMIT = 40.0;
            public static final double SUPPLY_CURRENT_LOWER_TIME = 0.0;

            // TODO: find this
            public static final double KP = 100.0;
            public static final double KI = 0.0;
            public static final double KD = 0.0;

            public static final double KS = 0.035; //0.045;
            public static final double KV = 0.7; //0.3;
            public static final double KG = 0.05; //0.06;
            public static final double KA = 0.035; //0.01;

            public static final GravityTypeValue GRAVITY_TYPE_VALUE = GravityTypeValue.Elevator_Static;

            // TODO: tune
            public static final double CRUISE_VELOCITY = 2.0;
            public static final double ACCELERATION = 7.5;
            public static final double JERK = 0.0;

            // public static final double TOP_SOFT_LIMIT = 1.0; // 100% range of motion
            public static final double BOTTOM_SOFT_LIMIT = 0.0; // 0% range of motion

            public static final double GROUND_OFFSET_M = 0.9;
            public static final double MAX_HEIGHT_M = 1.95;
        }

        public class Joint {
            public static final int MOTOR_ID = 11;
            public static final int CANCODER_ID = 11;

            // TODO: Tune new cancoder
            public static final double CANCODER_GEAR_RATIO = 28.0/80;
            public static final double CANCODER_OFFSET = -(.3596  - (0.25 / CANCODER_GEAR_RATIO));
            public static final double CANCODER_DISCONTINUITY_POINT = 0.5;

            public static final double CONVERSION_FACTOR = (120 / 12) * 15.0;

            // TODO: tune
            public static final double KS = 0.02;
            public static final double KG = 0.03;
            public static final double KV = 1.0;
            public static final double KA = 0.01;

            public static final double KP = 100.0; //100.0;
            public static final double KI = 0.0; //0.0;
            public static final double KD = 0.0; //0.0;

            public static final double CRUISE_VELOCITY = 0.3;
            public static final double ACCELERATION = 1.0;
            public static final double JERK = 0.0; //0.0;

            public static final GravityTypeValue GRAVITY_TYPE_VALUE = GravityTypeValue.Arm_Cosine;

            public static final double STATOR_CURRENT_LIMT = 30.0;
            public static final double SUPPLY_CURRENT_LIMIT = 30.0;
            public static final double SUPPLY_CURRENT_LOWER_TIME = 0.0;
        }

        public class Intake {
            public static final int MOTOR_ID = 12;
            public static final int FOLLOW_ID = 13;

            public static final double ROLLER_SPEED = 0.35; // 35%

            public static final int CURRENT_LIMIT = 25; // subject to change
        }

        public class Positions {
            // TODO: tune
            public static final double[] L4_SCORING_POS_M = { -0.1, 2.7 };
            public static final double[] L3_SCORING_POS_M = { -0.10, 1.55 };
            public static final double[] L2_SCORING_POS_M = { -0.0, 0.2 };
            public static final double[] L1_SCORING_POS_M = { 0.0, 0.0 };

            public static final double[] INTAKE_POS_M = { 0.50, 0.81 };

            public static final double[] HOME_POS_M = { 0.0, 0.0 };
        }
    }

    public class AlgaeIntake {
        public class Joint {
            public static final int MOTOR_ID = 14;
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
}