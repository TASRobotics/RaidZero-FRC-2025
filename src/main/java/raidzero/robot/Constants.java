package raidzero.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import com.ctre.phoenix6.signals.GravityTypeValue;

public class Constants {
    public class AlgaeIntake {
        public class Joint {
            public static final int MOTOR_ID = 14;
            public static final double CONVERSION_FACTOR = (54 / 30) * 9;

            public static final double INTAKE_POSITION = 0.0;
            public static final double HOME_POSITION = 0.3;

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

        public class Intake {
            public static final int MOTOR_ID = 15;
            public static final double CONVERSION_FACTOR = 45;

            public static final double INTAKE_SPEED = 0.2; // 20 %
        }
    }

    public class Bindings {
        public static final int CLIMB_UP = 1;
        public static final int CLIMB_DEPLOY = 2;
        public static final int CLIMB_DOWN = 3;

        public static final int ALGAE_INTAKE = 4;
        public static final int ALGAE_EXTAKE = 5;

        public static final int L1 = 6;
        public static final int L2 = 7;
        public static final int L3 = 8;
        public static final int L4 = 9;

        public static final int CORAL_EXTAKE = 10;
        public static final int CORAL_INTAKE = 11;
        public static final int CORAL_SCOOCH = 12;
    }

    public class CANdle {
        public static final int CAN_ID = 1;

        public static final double CLIMB_JOINT_THRESHOLD = 0.125;

        public static final double ARM_JOINT_LOWER_BOUND = 0.1675;
        public static final double ARM_JOINT_UPPER_BOUND = 0.1726;
    }

    public class Climb {
        public class Joint {
            public static final int MOTOR_ID = 17;
            public static final double SENSOR_TO_MECHANISM_RATIO = 80.0 / 10.0;

            public static final double KS = 0.02;
            public static final double KG = 0.04;
            public static final double KV = 1.2;
            public static final double KA = 0.1;

            public static final double KP = 15.0;
            public static final double KI = 0.0;
            public static final double KD = 0.0;

            public static final double HOME_POS = 0.25;
            public static final double DEPLOYED_POS = 0.0;

            public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;

            public static final double CURRENT_LIMIT = 20.0;
            public static final double SUPPLY_CURRENT_LIMIT = 20.0;
            public static final double SUPPLY_CURRENT_LOWER_TIME = 0.0;

            public static final double FORWARD_SOFT_LIMIT = 0.5;
            public static final double REVERSE_SOFT_LIMIT = -0.0;

            public static final double MOTION_MAGIC_CRUISE_VELOCITY = 0.2;
            public static final double MOTION_MAGIC_ACCELERATION = 1.0;
        }

        public class Winch {
            public static final int MOTOR_ID = 16;

            public static final double SPEED = 0.25;
        }
    }

    public class Swerve {
        public static enum REEFS {
            LEFT, RIGHT
        }

        public static final List<Pose2d> STATION_WAYPOINTS = new ArrayList<Pose2d>(
            List.of(
                new Pose2d(1.0746, 1.03, Rotation2d.fromDegrees(55)), // 12 Station
                new Pose2d(1.0492, 7.01, Rotation2d.fromDegrees(-55)) // 13 Station
            )
        );

        public static final List<Pose2d> LEFT_REEF_WAYPOINTS = new ArrayList<Pose2d>(
            List.of(
                new Pose2d(3.70, 3.16, Rotation2d.fromDegrees(60)), // 17 Left
                new Pose2d(3.30, 4.15, Rotation2d.fromDegrees(0)), // 18 Left
                new Pose2d(4.05, 5.1, Rotation2d.fromDegrees(300)), // 19 Left
                new Pose2d(5.2619, 4.99953, Rotation2d.fromDegrees(240)), // 20 Left
                new Pose2d(5.70, 3.85, Rotation2d.fromDegrees(180)), // 21 Left
                new Pose2d(4.9494, 2.88847, Rotation2d.fromDegrees(120)) // 22 Left
            )
        );

        public static final List<Pose2d> RIGHT_REEF_WAYPOINTS = new ArrayList<Pose2d>(
            List.of(
                new Pose2d(4.05, 2.95, Rotation2d.fromDegrees(60)), // 17 Right
                new Pose2d(3.30, 3.85, Rotation2d.fromDegrees(0)), // 18 Right
                new Pose2d(3.70, 4.89, Rotation2d.fromDegrees(300)), // 19 Right
                new Pose2d(4.9419, 5.16453, Rotation2d.fromDegrees(240)), // 20 Right
                new Pose2d(5.70, 4.20, Rotation2d.fromDegrees(180)), // 21 Right
                new Pose2d(5.2619, 3.05047, Rotation2d.fromDegrees(120)) // 22 Right
            )
        );
    }

    public class TelescopingArm {

        public class Intake {
            public static final int MOTOR_ID = 12;
            public static final int FOLLOW_ID = 13;

            public static final double INTAKE_SPEED = 0.08;
            public static final double INTAKE_LOWER_SPEED = 0.05;

            public static final double EXTAKE_SPEED = 0.1;
            public static final double EXTAKE_TIMEOUT_S = 1.0;

            public static final double LASERCAN_DISTANCE_THRESHOLD_MM = 50;

            public static final int CURRENT_LIMIT = 25; // subject to change
        }

        public class Joint {
            public static final int MOTOR_ID = 11;
            public static final int CANCODER_ID = 11;

            public static final double CANCODER_GEAR_RATIO = 28.0 / 80;
            public static final double CANCODER_OFFSET = -(.3596 - (0.25 / CANCODER_GEAR_RATIO));
            public static final double CANCODER_DISCONTINUITY_POINT = 0.5;

            public static final double CONVERSION_FACTOR = (120 / 12) * 20.0;

            public static final double KS = 0.02;
            public static final double KG = 0.03;
            public static final double KV = 1.0;
            public static final double KA = 0.01;

            public static final double KP = 150.0;
            public static final double KI = 0.0;
            public static final double KD = 0.0;

            public static final double CRUISE_VELOCITY = 0.2;
            public static final double ACCELERATION = 0.6;
            public static final double JERK = 0.0;

            public static final GravityTypeValue GRAVITY_TYPE_VALUE = GravityTypeValue.Arm_Cosine;

            public static final double STATOR_CURRENT_LIMT = 30.0;
            public static final double SUPPLY_CURRENT_LIMIT = 30.0;
            public static final double SUPPLY_CURRENT_LOWER_TIME = 0.0;
        }

        public class Positions {
            public static final double[] L4_SCORING_POS_M = { -0.1, 2.7 };
            public static final double[] L3_SCORING_POS_M = { -0.15, 1.55 };
            public static final double[] L2_SCORING_POS_M = { -0.0, 0.2 };
            public static final double[] L1_SCORING_POS_M = { 0.0, 0.0 };

            public static final double[] INTAKE_POS_M = { 0.5, 0.80 };

            public static final double[] HOME_POS_M = { 0.0, 0.0 };
        }

        public class Telescope {
            public static final int MOTOR_ID = 10;

            public static final double CONVERSION_FACTOR = 52.643555;

            public static final double STATOR_CURRENT_LIMIT = 40.0;
            public static final double SUPPLY_CURRENT_LIMIT = 40.0;
            public static final double SUPPLY_CURRENT_LOWER_TIME = 0.0;

            public static final double KP = 100.0;
            public static final double KI = 0.0;
            public static final double KD = 0.0;

            public static final double KS = 0.035; // 0.045;
            public static final double KV = 0.7; // 0.3;
            public static final double KG = 0.05; // 0.06;
            public static final double KA = 0.035; // 0.01;

            public static final GravityTypeValue GRAVITY_TYPE_VALUE = GravityTypeValue.Elevator_Static;

            public static final double CRUISE_VELOCITY = 1.5;
            public static final double ACCELERATION = 5.0;
            public static final double JERK = 0.0;

            // public static final double TOP_SOFT_LIMIT = 1.0; // 100% range of motion
            public static final double BOTTOM_SOFT_LIMIT = 0.0; // 0% range of motion

            public static final double GROUND_OFFSET_M = 0.9;
            public static final double MAX_HEIGHT_M = 1.95;
        }
    }

    public static final String CANIVORE_NAME = "CANdoAttitude";

    public static final double STICK_DEADBAND = 0.2;
}