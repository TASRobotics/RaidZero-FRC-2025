package raidzero.robot;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.ArrayList;
import java.util.List;

public class Constants {
    public static class Bindings {
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

        public static final int TOP_RIGHT = 13;
        public static final int BOTTOM_RIGHT = 14;
        public static final int BOTTOM_LEFT = 15;
        public static final int TOP_LEFT = 16;
    }

    public static class CANdle {
        public static final int CAN_ID = 1;

        public static final double CLIMB_JOINT_THRESHOLD = 0.125;

        public static final double ARM_JOINT_LOWER_BOUND = 0.183;
        public static final double ARM_JOINT_UPPER_BOUND = 0.188;
    }

    public static class Climb {
        public static class Joint {
            public static final int MOTOR_ID = 17;
            public static final double SENSOR_TO_MECHANISM_RATIO = 80.0 / 10.0;

            public static final double KS = 0.02;
            public static final double KG = 0.08;
            public static final double KV = 1.2;
            public static final double KA = 0.1;

            public static final double KP = 20.0;
            public static final double KI = 0.0;
            public static final double KD = 0.0;

            public static final double HOME_POS = 0.25;
            public static final double DEPLOYED_POS = 0.0;

            public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;

            public static final double CURRENT_LIMIT = 80.0;
            public static final double SUPPLY_CURRENT_LIMIT = 80.0;
            public static final double SUPPLY_CURRENT_LOWER_TIME = 0.0;

            public static final double FORWARD_SOFT_LIMIT = 0.5;
            public static final double REVERSE_SOFT_LIMIT = -0.0;

            public static final double MOTION_MAGIC_CRUISE_VELOCITY = 0.2;
            public static final double MOTION_MAGIC_ACCELERATION = 1.0;
        }

        public static class Winch {
            public static final int MOTOR_ID = 16;

            public static final double SPEED = 1.0;
        }
    }

    public static class Swerve {
        public static enum REEFS {
            LEFT, RIGHT
        }

        public static final List<Pose2d> STATION_WAYPOINTS = new ArrayList<Pose2d>(
            List.of(
                new Pose2d(1.151, 1.03, Rotation2d.fromDegrees(55)), // 12 Station
                new Pose2d(1.1383, 7.01, Rotation2d.fromDegrees(-55)) // 13 Station 1.0873
            )
        );

        public static final List<Pose2d> LEFT_REEF_WAYPOINTS = new ArrayList<Pose2d>(
            List.of(
                new Pose2d(3.735, 3.14, Rotation2d.fromDegrees(60)), // 17 Left
                new Pose2d(3.30, 4.15, Rotation2d.fromDegrees(0)), // 18 Left
                new Pose2d(4.06, 5.105, Rotation2d.fromDegrees(300)), // 19 Left
                new Pose2d(5.2619, 4.99953, Rotation2d.fromDegrees(240)), // 20 Left
                new Pose2d(5.70, 3.85, Rotation2d.fromDegrees(180)), // 21 Left
                new Pose2d(4.9113, 2.93927, Rotation2d.fromDegrees(120)) // 22 Left
            )
        );

        public static final List<Pose2d> RIGHT_REEF_WAYPOINTS = new ArrayList<Pose2d>(
            List.of(
                new Pose2d(4.05, 2.95, Rotation2d.fromDegrees(60)), // 17 Right
                new Pose2d(3.30, 3.85, Rotation2d.fromDegrees(0)), // 18 Right
                new Pose2d(3.713, 4.925, Rotation2d.fromDegrees(300)), // 19 Right
                new Pose2d(4.9489, 5.16, Rotation2d.fromDegrees(240)), // 20 Right
                new Pose2d(5.70, 4.20, Rotation2d.fromDegrees(180)), // 21 Right
                new Pose2d(5.2619, 3.05047, Rotation2d.fromDegrees(120)) // 22 Right
            )
        );

        public static final Pose2d BLUE_PROCESSOR = new Pose2d(5.987542, 0.78, Rotation2d.fromDegrees(90));
        public static final Pose2d RED_PROCESSOR = new Pose2d(17.55 - 5.987542, 8.05 - 0.78, Rotation2d.fromDegrees(180));
    }

    public static class TelescopingArm {
        public static class Intake {
            public static final int LEADER_ID = 12;
            public static final int FOLLOWER_ID = 13;

            public static final int TOP_LASERCAN = 0;
            public static final int BOTTOM_LASERCAN = 1;

            public static final MotorArrangementValue MOTOR_ARRANGEMENT = MotorArrangementValue.Minion_JST;

            public static final InvertedValue INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;

            public static final int STATOR_CURRENT_LIMIT = 50;
            public static final int SUPPLY_CURRENT_LIMIT = 50;
            public static final double SUPPLY_CURRENT_LOWER_TIME = 0.0;

            public static final double TOP_LASER_THRESHOLD_MM = 50.0;
            public static final double BOTTOM_LASER_THRESHOLD_MM = 100.0;

            public static final double INTAKE_SPEED = 0.85;
            public static final double LOWER_SPEED = 0.25;
            public static final double EJECT_SPEED = -0.80;
            public static final double REVERSE_SPEED = -0.2;

            public static final double STALL_CURRENT_THRESHOLD = 20.0;
            public static final double CURRENT_SPIKE_THRESHOLD = 10.0;

            public static final double EXTAKE_SPEED = 1.0;
            public static final double EXTAKE_TIMEOUT_S = 1.0;

            public static final double ALGAE_INTAKE_SPEED = 1.0;
            public static final double ALGAE_EJECT_SPEED = -1.0;
            public static final double HOLD_SPEED = 0.1;

            public static final double KP = 1.0;
            public static final double KI = 0.0;
            public static final double KD = 0.0;
        }

        public static class Joint {
            public static final int MOTOR_ID = 11;
            public static final int CANCODER_ID = 11;

            public static final double CANCODER_GEAR_RATIO = 28.0 / 80.0;
            public static final double CANCODER_OFFSET = -(0.348633 - (0.25 / CANCODER_GEAR_RATIO));
            public static final double CANCODER_DISCONTINUITY_POINT = 0.5;
            public static final double SENSOR_TO_MECHANISM_RATIO = 200.0;

            public static final double CONVERSION_FACTOR = (120.0 / 12.0) * 20.0;

            public static final double KS = 0.02;
            public static final double KG = 0.03;
            public static final double KV = 1.0;
            public static final double KA = 0.01;

            public static final double KP = 160.0;
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

        public static class Positions {
            public static final double[] L4_SCORING_POS_M = { -0.1692, 2.75 };
            public static final double[] L4_SCORING_POS_M_BLUE = { -0.1692, 2.68 };
            public static final double[] L4_CHECK_POSITION = { -0.25, 2.62 };
            public static final double[] L4_GRAND_SLAM = { -0.2, 1.57 };

            public static final double[] L3_SCORING_POS_M = { -0.20, 1.57 };
            public static final double[] L2_SCORING_POS_M = { -0.15, 0.9 };
            public static final double[] L1_SCORING_POS_M = { 0.0, 0.0 };

            public static final double[] INTAKE_POS_M = { 0.5, 0.8 };
            public static final double[] INTAKE_POS_M_BLUE = { 0.5, 0.8 };

            public static final double[] L3_ALGAE_POS_M = { 0.75, 1.3 };
            public static final double[] L2_ALGAE_POS_M = { 0.6, 0.7 };
            public static final double[] BARGE_SCORE_POS_M = { 0, 2.8 };

            public static double[] HOME_POS_M = { 0.0, 0.0 };
        }

        public static class Telescope {
            public static final int MOTOR_ID = 10;

            public static final double CONVERSION_FACTOR = 52.643555;

            public static final double STATOR_CURRENT_LIMIT = 40.0;
            public static final double SUPPLY_CURRENT_LIMIT = 40.0;
            public static final double SUPPLY_CURRENT_LOWER_TIME = 0.0;

            public static final double KP = 110.0;
            public static final double KI = 0.0;
            public static final double KD = 0.0;

            public static final double KS = 0.035;
            public static final double KV = 0.7;
            public static final double KG = 0.05;
            public static final double KA = 0.035;

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

    public static final String BASE_CANIVORE = "CANdoAttitude";
    public static final String KAYNE_BUS = "Kaynebus";
    public static final String RIO_BUS = "rio";

    public static final double STICK_DEADBAND = 0.2;
}