package raidzero.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {

    public class Swerve {
        public static final double STICK_DEADBAND = 0.2;

        public static enum REEFS {
            LEFT, RIGHT
        }

        public static final List<Pose2d> STATION_WAYPOINTS = new ArrayList<Pose2d>(List.of(
            new Pose2d(1.10, 0.98, Rotation2d.fromDegrees(55)), // 12 Station
            new Pose2d(1.10, 7.07, Rotation2d.fromDegrees(-55)) // 13 Station
        ));

        public static final List<Pose2d> LEFT_REEF_WAYPOINTS = new ArrayList<Pose2d>(List.of(
            new Pose2d(3.70, 3.16, Rotation2d.fromDegrees(60)), // 17 Left
            new Pose2d(3.30, 3.85, Rotation2d.fromDegrees(0)), // 18 Left
            new Pose2d(4.05, 5.1, Rotation2d.fromDegrees(300)), // 19 Left
            new Pose2d(5.2619, 4.99953, Rotation2d.fromDegrees(240)), // 20 Left
            new Pose2d(5.70, 3.88, Rotation2d.fromDegrees(180)), // 21 Left
            new Pose2d(4.9419, 2.88547, Rotation2d.fromDegrees(120)) // 22 Left
        ));

        public static final List<Pose2d> RIGHT_REEF_WAYPOINTS = new ArrayList<Pose2d>(List.of(
            new Pose2d(4.05, 2.95, Rotation2d.fromDegrees(60)), // 17 Right
            new Pose2d(3.30, 4.15, Rotation2d.fromDegrees(0)),  // 18 Right
            new Pose2d(3.70, 4.89, Rotation2d.fromDegrees(300)), // 19 Right
            new Pose2d(4.9419, 5.16453, Rotation2d.fromDegrees(240)), // 20 Right
            new Pose2d(5.70, 4.20, Rotation2d.fromDegrees(180)), // 21 Right
            new Pose2d(5.2619, 3.05047, Rotation2d.fromDegrees(120))  // 22 Right
        ));
    }

    public class TelescopingArm {
        public class Telescope {
            public static final int MOTOR_ID = 0;
            public static final double CONVERSION_FACTOR = 0.022 * Math.PI;

            public static final double MAX_LENGTH_M = 1.8288;
            public static final double MIN_LENGTH_M = 0.0;

            public static final double KS = 0.0;
            public static final double KV = 0.0;
            public static final double KA = 0.0;
            public static final double KP = 0.0;
            public static final double KI = 0.0;
            public static final double KD = 0.0;

            public static final double CRUISE_VELOCITY = 0.0;
            public static final double ACCELERATION = 0.0;
            public static final double JERK = 0.0;

            // 2cm of tolerance
            public static final double POSITION_TOLERANCE_ROTATIONS = 0.02 * CONVERSION_FACTOR;
        }

        public class ArmJoint {
            public static final int MOTOR_ID = 1;
            public static final double CONVERSION_FACTOR = 1.0;
            public static final double LENGTH_M = 1.0 * CONVERSION_FACTOR;

            public static final double KS = 0.0;
            public static final double KV = 0.0;
            public static final double KA = 0.0;
            public static final double KP = 0.0;
            public static final double KI = 0.0;
            public static final double KD = 0.0;

            public static final double CRUISE_VELOCITY = 0.0;
            public static final double ACCELERATION = 0.0;
            public static final double JERK = 0.0;

            // 2cm of tolerance
            public static final double POSITION_TOLERANCE_ROTATIONS = 0.02 * CONVERSION_FACTOR;
        }

        public class Roller {
            public static final int MOTOR_ID = 0;

            public static final double ROLLER_SPEED = 0.2; // 2%
        }
    }
}