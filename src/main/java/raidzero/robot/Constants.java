package raidzero.robot;

import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.ArrayList;
import java.util.List;

public class Constants {
    public static class AlgaeIntake {
        public static class Joint {
            public static final int MOTOR_ID = 14;
            public static final double CONVERSION_FACTOR = (54.0 / 30.0) * 9.0;

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

        public static class Intake {
            public static final int MOTOR_ID = 15;
            public static final double CONVERSION_FACTOR = 45.0;

            public static final double INTAKE_SPEED = 0.2;
        }
    }

    public static class Bindings {
        public static class Digital {
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

        public static class Analog {
            public static final int ALGAE_EXTAKE = 0;
        }
    }

    public static class CANdle {
        public static final int CAN_ID = 1;

        public static final double CLIMB_JOINT_THRESHOLD = 0.125;

        public static final double ARM_JOINT_LOWER_BOUND = 0.1754;
        public static final double ARM_JOINT_UPPER_BOUND = 0.1805;
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

            public static final double SPEED = 0.75;
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
                new Pose2d(3.70, 3.16, Rotation2d.fromDegrees(60)), // 17 Left
                new Pose2d(3.30, 4.15, Rotation2d.fromDegrees(0)), // 18 Left
                new Pose2d(4.05, 5.1, Rotation2d.fromDegrees(300)), // 19 Left
                new Pose2d(5.2619, 4.99953, Rotation2d.fromDegrees(240)), // 20 Left
                new Pose2d(5.70, 3.85, Rotation2d.fromDegrees(180)), // 21 Left
                new Pose2d(4.9113, 2.93927, Rotation2d.fromDegrees(120)) // 22 Left
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

    public static class TelescopingArm {
        public static class Intake {
            public static final int MOTOR_ID = 12;
            public static final int FOLLOW_ID = 13;

            public static final double INTAKE_SPEED = 0.25;
            public static final double INTAKE_LOWER_SPEED = 0.05;

            public static final double EXTAKE_SPEED = 0.1;
            public static final double EXTAKE_TIMEOUT_S = 1.0;

            public static final double LASERCAN_DISTANCE_THRESHOLD_MM = 50.0;

            public static final int CURRENT_LIMIT = 25;

            public static final int BOTTOM_LASERCAN_ID = 0;
            public static final int TOP_LASERCAN_ID = 1;

            public static final int SERVO_HUB_ID = 3;

            public static final int SERVO_RETRACTED = 1950;
            public static final int SERVO_EXTENDED = 1300;
            public static final int SERVO_CENTER_WIDTH = 1625;
        }

        public static class Joint {
            public static final int MOTOR_ID = 11;
            public static final int CANCODER_ID = 11;

            public static final double CANCODER_GEAR_RATIO = 28.0 / 80.0;
            public static final double CANCODER_OFFSET = -(0.358398 - (0.25 / CANCODER_GEAR_RATIO));
            public static final double CANCODER_DISCONTINUITY_POINT = 0.5;

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
            public static double[] L4_SCORING_POS_M = { -0.24, 2.72 };
            public static double[] L4_SCORING_POS_M_BLUE = { -0.17, 2.68 };
            public static double[] L4_CHECK_POSITION = { -0.25, 2.62 };
            public static double[] L4_GRAND_SLAM = { -0.2, 1.57 };

            public static double[] L3_SCORING_POS_M = { -0.25, 1.57 };
            public static double[] L2_SCORING_POS_M = { -0.2, 0.9 };
            public static double[] L1_SCORING_POS_M = { 0.0, 0.0 };

            public static double[] INTAKE_POS_M = { 0.5, 0.835 };
            public static double[] INTAKE_POS_M_BLUE = { 0.5, 0.81 };

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

    public static class AlgaeDescore {
        public static class Extake {
            public static final int MOTOR_ID = 0;

            public static final double CONVERSION_FACTOR = 1.0;
            public static final double EXTAKE_SPEED = 0.0;
        }

        public static class Telescope {
            public static final int MOTOR_ID = 18;

            public static final double CONVERSION_FACTOR = 1.0;

            public static final double EXTAKE_POSITION = 0.0;
            public static final double HOME_POSITION = 0.0;

            public static final double STATOR_CURRENT_LIMIT = 10.0;
            public static final double SUPPLY_CURRENT_LIMIT = 10.0;
            public static final double SUPPLY_CURRENT_LOWER_TIME = 0.0;

            public static final double TOP_SOFT_LIMIT = 0.0;
            public static final double BOTTOM_SOFT_LIMIT = 0.0;

            public static final double KP = 0.0;
            public static final double KI = 0.0;
            public static final double KD = 0.0;

            public static final double KS = 0.0;
            public static final double KV = 0.0;
            public static final double KG = 0.0;
            public static final double KA = 0.0;

            public static final GravityTypeValue GRAVITY_TYPE_VALUE = GravityTypeValue.Elevator_Static;

            public static final double CRUISE_VELOCITY = 0.0;
            public static final double ACCELERATION = 0.0;
            public static final double JERK = 0.0;
        }
    }

    public static final String CANIVORE_NAME = "CANdoAttitude";

    public static final double STICK_DEADBAND = 0.2;
}