package raidzero.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import raidzero.robot.subsystems.drivetrain.TunerConstants.TunerSwerveDrivetrain;
import raidzero.robot.Constants;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class Swerve extends TunerSwerveDrivetrain implements Subsystem {
    private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms
    private Notifier simNotifier = null;
    private double lastSimTime;

    // Blue alliance sees forward as 0 degrees (toward red alliance wall)
    private static final Rotation2d BLUE_ALLLIANCE_PERSPECTIVE_ROTATION = Rotation2d.kZero;
    // Red alliance sees forward as 180 degrees (toward blue alliance wall)
    private static final Rotation2d RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.k180deg;
    private boolean hasAppliedOperatorPerspective = false;

    // Swerve requests to apply during SysId characterization
    private final SwerveRequest.SysIdSwerveTranslation translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    private final SwerveRequest.ApplyRobotSpeeds pathplannerSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    private StructArrayPublisher<SwerveModuleState> modulePublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    private StructPublisher<Pose2d> botpose = NetworkTableInstance.getDefault().getStructTopic("botPoseNT", Pose2d.struct).publish();

    private final Field2d field = new Field2d();

    private boolean waypointsTransformed = false;

    private static Swerve system;

    /*
     * SysId routine for characterizing translation. This is used to find PID
     * gains
     * for the drive motors.
     */
    private final SysIdRoutine sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null, // Default: 1 V/s
            Volts.of(4), // Reduce dynamic to 4 V to prevent brownout
            null, // Default: 10 s
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(this.translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing steer. This is used to find PID gains
     * for
     * the steer motors.
     */
    private final SysIdRoutine sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null, // Default: 1 V/s
            Volts.of(7), // Dynamic: 7 V
            null, // Default: 10 s
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(this.steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle
     * HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
     * importing the log to SysId.
     */
    private final SysIdRoutine sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(Math.PI / 6).per(Second), // Rad/s but SysId only supports V/s
            Volts.of(Math.PI), // Rad/s but SysId only supports V
            null, // Default: 10 s
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                setControl(this.rotationCharacterization.withRotationalRate(output.in(Volts))); // Rad/s but SysId only supports V
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts)); // Log output for SysId
            },
            null,
            this
        )
    );

    // The SysId routine to test
    private SysIdRoutine sysIdRoutineToApply = sysIdRoutineSteer;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public Swerve(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);

        SmartDashboard.putData("Field", field);

        configureAutoBuilder();
        initializeOtf();

        if (Utils.isSimulation()) {
            this.startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public Swerve(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);

        SmartDashboard.putData("Field", field);

        configureAutoBuilder();
        initializeOtf();

        if (Utils.isSimulation()) {
            this.startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve
     *                                  drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz
     *                                  on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public Swerve(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency, Matrix<N3, N1> odometryStandardDeviation, Matrix<N3, N1> visionStandardDeviation, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);

        SmartDashboard.putData("Field", field);

        configureAutoBuilder();
        initializeOtf();

        if (Utils.isSimulation()) {
            this.startSimThread();
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return this.sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return this.sysIdRoutineToApply.dynamic(direction);
    }

    /**
     * Uses PathPlanner's {@link AutoBuilder#pathfindToPose} to move to the desired pose
     * 
     * @param pose The desired pose
     * @return A {@link DeferredCommand} that moves the robot to the desired pose
     */
    public Command goToPose(Pose2d pose) {
        return defer(
            () -> AutoBuilder.pathfindToPose(
                pose,
                new PathConstraints(
                    3.5,
                    4.0,
                    Units.degreesToRadians(540),
                    Units.degreesToRadians(720)
                )
            ).finallyDo((interrupted) -> this.stop())
        );
    }

    /**
     * Moves the robot to the nearest left or right reef
     * 
     * @param reef Desired left or right reef from {@link Constants.Swerve.REEFS}
     * @return A {@link DeferredCommand} that moves the robot to the nearest left or right reef
     */
    public Command pathToReef(Constants.Swerve.REEFS reef) {
        return defer(() -> {
            Pose2d target = null;

            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                target = this.getState().Pose.nearest(
                    (reef == Constants.Swerve.REEFS.LEFT) ? Constants.Swerve.RIGHT_REEF_WAYPOINTS : Constants.Swerve.LEFT_REEF_WAYPOINTS
                );
            } else {
                target = this.getState().Pose.nearest(
                    (reef == Constants.Swerve.REEFS.LEFT) ? Constants.Swerve.LEFT_REEF_WAYPOINTS : Constants.Swerve.RIGHT_REEF_WAYPOINTS
                );
            }

            return goToPose(target).withTimeout(0.01).andThen(goToPose(target));
        });
    }

    /**
     * Moves the robot to the nearest coral station
     * 
     * @return A {@link DeferredCommand} that moves the robot to the nearest coral station
     */
    public Command pathToStation() {
        return defer(() -> {
            Pose2d target = this.getState().Pose.nearest(Constants.Swerve.STATION_WAYPOINTS);

            return goToPose(target).withTimeout(0.01).andThen(goToPose(target));
        });
    }

    /**
     * Returns a {@link BooleanSupplier} that checks if the arm is able to be deployed
     * 
     * @return A {@link BooleanSupplier} that checks if the arm is able to be deployed
     */
    public BooleanSupplier isArmDeployable() {
        return () -> {
            Translation2d currTranslation = this.getState().Pose.getTranslation();
            ChassisSpeeds currSpeeds = this.getState().Speeds;

            return currTranslation.getDistance(
                this.getState().Pose.nearest(Constants.Swerve.STATION_WAYPOINTS).getTranslation()
            ) > 1.25 &&
                (currTranslation.getX() < 7.525 ||
                    currTranslation.getX() > 10.025) &&
                (Math.sqrt(Math.pow(currSpeeds.vxMetersPerSecond, 2) + Math.pow(currSpeeds.vyMetersPerSecond, 2)) < 2.0);
        };
    }

    /**
     * Returns a {@link BooleanSupplier} that checks if the robot is not in a climb zone
     * 
     * @return A {@link BooleanSupplier} that checks if the robot is not in a climb zone
     */
    public BooleanSupplier isNotInClimbZone() {
        return () -> {
            Translation2d currTranslation = this.getState().Pose.getTranslation();

            return currTranslation.getX() > 7.525 && currTranslation.getX() < 10.025;
        };
    }

    /**
     * <ul>
     * <li>Transforms each waypoint in the provided list for the red alliance
     * <li><b><em>This method modifies the points within the list
     * </ul>
     * 
     * @param waypoints The list of Pose2d waypoints (defined in blue origin coordinates)
     */
    private void transformWaypointsForAlliance(List<Pose2d> waypoints) {
        final double FIELD_LENGTH = 17.55;
        final double X_OFFSET = 0.0;

        for (int i = 0; i < waypoints.size(); i++) {
            Pose2d bluePose = waypoints.get(i);
            waypoints.set(
                i,
                new Pose2d(
                    FIELD_LENGTH - bluePose.getX() + X_OFFSET,
                    bluePose.getY(),
                    Rotation2d.fromDegrees(180 - bluePose.getRotation().getDegrees())
                )
            );
        }
    }

    /**
     * Stops the swerve
     * 
     * @return A {@link Command} to stop the swerve
     */
    public Command stop() {
        return runOnce(
            () -> this.setControl(
                (new SwerveRequest.RobotCentric()).withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0)
            )
        );
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should
         * apply
         * it regardless of DS state.
         * This allows us to correct the perspective in case the robot code
         * restarts
         * mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled.
         * This ensures driving behavior doesn't change until an explicit
         * disable event
         * occurs during testing.
         */
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red ?
                        RED_ALLIANCE_PERSPECTIVE_ROTATION :
                        BLUE_ALLLIANCE_PERSPECTIVE_ROTATION
                );
                this.hasAppliedOperatorPerspective = true;
            });
        }

        modulePublisher.set(this.getState().ModuleStates);
        botpose.set(this.getState().Pose);
        field.setRobotPose(this.getState().Pose);
    }

    /**
     * Starts the simulation thread.
     */
    private void startSimThread() {
        this.lastSimTime = Utils.getCurrentTimeSeconds();

        /*
         * Run simulation at a faster rate so PID gains behave more reasonably
         */
        this.simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });

        this.simNotifier.startPeriodic(SIM_LOOP_PERIOD);
    }

    /**
     * <ul>
     * <li>Initializes the on-the-fly (OTF) waypoints for the robot
     * <li>Transforms the waypoints for the red alliance if needed
     * </ul>
     */
    public void initializeOtf() {
        if ((DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) && !waypointsTransformed) {
            waypointsTransformed = true;

            transformWaypointsForAlliance(Constants.Swerve.STATION_WAYPOINTS);
            transformWaypointsForAlliance(Constants.Swerve.LEFT_REEF_WAYPOINTS);
            transformWaypointsForAlliance(Constants.Swerve.RIGHT_REEF_WAYPOINTS);
        }
    }

    /**
     * Configures the AutoBuilder for the Swerve subsystem
     */
    private void configureAutoBuilder() {
        try {
            AutoBuilder.configure(
                () -> this.getState().Pose,
                this::resetPose,
                () -> this.getState().Speeds,
                (speeds, feedforwards) -> setControl(
                    pathplannerSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    new PIDConstants(3.3, 0, 0),
                    new PIDConstants(3, 0, 0)
                ),
                RobotConfig.fromGUISettings(),
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this
            );
        } catch (Exception e) {
            DriverStation.reportError("Failed to load PathPlanner config and configure Autobuilder", e.getStackTrace());
        }
    }

    /**
     * Gets the {@link Swerve} subsystem instance
     * 
     * @return The {@link Swerve} subsystem instance
     */
    public static Swerve system() {
        if (system == null) {
            system = new Swerve(
                TunerConstants.DrivetrainConstants,
                TunerConstants.FrontLeft,
                TunerConstants.FrontRight,
                TunerConstants.BackLeft,
                TunerConstants.BackRight
            );
        }

        return system;
    }
}