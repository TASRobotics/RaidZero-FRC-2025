// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package raidzero.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import raidzero.robot.subsystems.LEDStrip.ArmStrip;
import raidzero.robot.subsystems.algaedescore.AlgaeExtake;
import raidzero.robot.subsystems.algaedescore.Telescope;
import raidzero.robot.subsystems.climb.ClimbJoint;
import raidzero.robot.subsystems.climb.Winch;
import raidzero.robot.subsystems.drivetrain.Limelight;
import raidzero.robot.subsystems.drivetrain.Swerve;
import raidzero.robot.subsystems.drivetrain.TunerConstants;
import raidzero.robot.subsystems.telescopingarm.*;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SlewRateLimiter slewRateLimiter = new SlewRateLimiter(2);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandGenericHID operator = new CommandGenericHID(1);

    public final Swerve swerve = Swerve.system();

    public final Arm arm = Arm.system();
    public final CoralIntake coralIntake = CoralIntake.system();

    public final Telescope algaeTelescope = Telescope.system();
    public final AlgaeExtake algaeExtake = AlgaeExtake.system();

    public final Limelight limes = Limelight.system();

    // public final AlgaeJoint algaeIntake = AlgaeJoint.system();

    public final ArmStrip armStrip = ArmStrip.system();

    public final ClimbJoint climbJoint = ClimbJoint.system();
    public final Winch climbWinch = Winch.system();

    public final SendableChooser<Command> autoChooser;

    /**
     * Constructs a {@link RobotContainer} instance
     */
    public RobotContainer() {
        registerPathplannerCommands();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("AutoChooser", autoChooser);

        configureBindings();
        PathfindingCommand.warmupCommand().schedule();

        climbJoint.setPosition(Constants.Climb.Joint.HOME_POS);
    }

    /**
     * Configures button bindings for the robot
     */
    private void configureBindings() {
        swerve.setDefaultCommand(
            swerve.applyRequest(
                () -> fieldCentricDrive
                    .withVelocityX(-joystick.getLeftY() * MaxSpeed * 0.67 * (arm.isUp() ? 0.3 : 1.0))
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * 0.67 * (arm.isUp() ? 0.3 : 1.0))
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
        );

        // arm.setDefaultCommand(arm.moveArmWithDelay(Constants.TelescopingArm.Positions.INTAKE_POS_M));
        arm.setDefaultCommand(arm.moveToIntake());
        algaeTelescope.setDefaultCommand(algaeTelescope.moveToHome());

        coralIntake.setDefaultCommand(coralIntake.stop());
        algaeExtake.setDefaultCommand(algaeExtake.stop());

        // algaeIntake.setDefaultCommand(algaeIntake.moveJoint(Constants.AlgaeIntake.Joint.HOME_POSITION));

        climbJoint.setDefaultCommand(climbJoint.run(Constants.Climb.Joint.HOME_POS));
        climbWinch.setDefaultCommand(climbWinch.stop());

        // * Driver controls
        joystick.rightTrigger().whileTrue(
            swerve.applyRequest(
                () -> fieldCentricDrive
                    .withVelocityX(-joystick.getLeftY() * MaxSpeed * (arm.isUp() ? 0.3 : 1.0))
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * (arm.isUp() ? 0.3 : 1.0))
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
        );

        joystick.rightTrigger().onTrue(new InstantCommand(() -> armStrip.setStrobeInterval(0.15)));
        joystick.rightTrigger().negate().onTrue(new InstantCommand(() -> armStrip.setStrobeInterval(0.5)));

        joystick.a().whileTrue(
            swerve.applyRequest(
                () -> robotCentricDrive.withVelocityY(slewRateLimiter.calculate(-joystick.getLeftX()) * MaxSpeed * 0.3)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
        );

        joystick.leftBumper().whileTrue(coralIntake.extake());
        joystick.rightBumper().onTrue(coralIntake.intake());

        joystick.b().whileTrue(
            swerve.pathToStation()
        );

        joystick.x().whileTrue(
            swerve.pathToReef(Constants.Swerve.REEFS.LEFT)
        );

        joystick.y().whileTrue(
            swerve.pathToReef(Constants.Swerve.REEFS.RIGHT)
        );

        joystick.povRight().whileTrue(
            swerve.applyRequest(
                () -> new SwerveRequest.SwerveDriveBrake()
            )
        );

        joystick.povDown().whileTrue(
            arm.moveWithDelay(Constants.TelescopingArm.Positions.INTAKE_POS_M_BLUE)
        );

        // * Operator controls
        operator.button(Constants.Bindings.Digital.TOP_LEFT).onTrue(new InstantCommand(() -> arm.decreaseIntakeYOffset(0.01), arm));
        operator.button(Constants.Bindings.Digital.BOTTOM_LEFT).onTrue(new InstantCommand(() -> arm.decreaseIntakeYOffset(-0.01), arm));
        operator.button(Constants.Bindings.Digital.TOP_RIGHT).onTrue(new InstantCommand(() -> arm.removeIntakeOffset(), arm));

        operator.button(Constants.Bindings.Digital.L2).whileTrue(
            arm.moveTo(Constants.TelescopingArm.Positions.L2_SCORING_POS_M)
                .onlyIf(swerve.isArmDeployable())
        );
        operator.button(Constants.Bindings.Digital.L3).whileTrue(
            arm.moveTo(Constants.TelescopingArm.Positions.L3_SCORING_POS_M)
                .onlyIf(swerve.isArmDeployable())
        );

        operator.button(Constants.Bindings.Digital.L4).negate().whileTrue(
            arm.moveToL4()
                .onlyIf(swerve.isArmDeployable())
        );

        operator.button(Constants.Bindings.Digital.ALGAE_INTAKE).onTrue(coralIntake.contingencyIntake());

        operator.button(Constants.Bindings.Digital.CORAL_EXTAKE).whileTrue(coralIntake.extake());
        operator.button(Constants.Bindings.Digital.CORAL_INTAKE).onTrue(coralIntake.intake());
        operator.button(Constants.Bindings.Digital.CORAL_SCOOCH).whileTrue(coralIntake.run(-Constants.TelescopingArm.Intake.EXTAKE_SPEED));

        operator.button(Constants.Bindings.Digital.BOTTOM_RIGHT).onTrue(coralIntake.unstuckServo());

        operator.button(Constants.Bindings.Digital.CLIMB_DEPLOY)
            .onTrue(
                Commands.waitSeconds(0.2)
                    .andThen(
                        climbJoint.run(Constants.Climb.Joint.DEPLOYED_POS)
                            .until(() -> operator.button(Constants.Bindings.Digital.CLIMB_UP).getAsBoolean())
                            .andThen(() -> climbJoint.stop()).alongWith(
                                new InstantCommand(
                                    () -> climbJoint.setDeployedState()
                                )
                            )
                    )
            );
        operator.button(Constants.Bindings.Digital.CLIMB_DEPLOY).onTrue(arm.climbPos());

        // operator.button(Constants.Bindings.Digital.CLIMB_UP)
        // .whileTrue(climbWinch.run(Constants.Climb.Winch.SPEED).onlyIf(climbJoint.isDeployed()));

        operator.button(Constants.Bindings.Digital.CLIMB_UP).whileTrue(climbWinch.run(Constants.Climb.Winch.SPEED));
        operator.button(Constants.Bindings.Digital.CLIMB_UP).onTrue(climbJoint.retract());

        operator.button(Constants.Bindings.Digital.CLIMB_DOWN)
            .whileTrue(climbWinch.run(-Constants.Climb.Winch.SPEED).onlyIf(climbJoint.isDeployed()));

        operator.axisGreaterThan(Constants.Bindings.Analog.ALGAE_EXTAKE, 0.6)
            .whileTrue(
                algaeTelescope.moveToExtake()
                    .alongWith(algaeExtake.extake(Constants.AlgaeDescore.Extake.EXTAKE_SPEED))
            );
        swerve.registerTelemetry(logger::telemeterize);
    }

    /**
     * Registers PathPlanner commands
     */
    private void registerPathplannerCommands() {
        NamedCommands.registerCommand(
            "ArmIntakeCoral",
            arm.moveWithDelay(Constants.TelescopingArm.Positions.INTAKE_POS_M)
                .withTimeout(0.75)
        );
        NamedCommands.registerCommand(
            "ArmIntakeCoralBLUE",
            arm.moveWithDelay(Constants.TelescopingArm.Positions.INTAKE_POS_M_BLUE)
                .withTimeout(0.75)
        );
        NamedCommands.registerCommand(
            "ArmL3",
            arm.moveTo(Constants.TelescopingArm.Positions.L3_SCORING_POS_M)
                .withTimeout(0.75)
        );
        NamedCommands.registerCommand(
            "ArmL4",
            arm.moveTo(Constants.TelescopingArm.Positions.L4_SCORING_POS_M)
                .withTimeout(0.75)
        );
        NamedCommands.registerCommand(
            "ArmL4BLUE",
            arm.moveTo(Constants.TelescopingArm.Positions.L4_SCORING_POS_M_BLUE)
                .withTimeout(0.75)
        );

        NamedCommands.registerCommand(
            "Slam",
            arm.moveWithoutDelay(Constants.TelescopingArm.Positions.L4_GRAND_SLAM).onlyIf(() -> arm.isUp())
                .until(
                    () -> arm.getJointPosition() >= arm.calculateJointAngle(Constants.TelescopingArm.Positions.L4_GRAND_SLAM) &&
                        arm.getTelescopePosition() <= arm.calculateTelescopeHeight(Constants.TelescopingArm.Positions.L4_GRAND_SLAM)
                )
                .withTimeout(0.5)
                .andThen(
                    arm.moveWithDelay(Constants.TelescopingArm.Positions.INTAKE_POS_M)
                        .withTimeout(0.75)
                )
        );

        NamedCommands.registerCommand(
            "ExtakeCoral",
            coralIntake.run(0.1).until(
                () -> {
                    return coralIntake.getBottomLaserDistance() >= Constants.TelescopingArm.Intake.LASERCAN_DISTANCE_THRESHOLD_MM &&
                        coralIntake.getTopLaserDistance() >= Constants.TelescopingArm.Intake.LASERCAN_DISTANCE_THRESHOLD_MM;
                }
            ).withTimeout(1.0).andThen(() -> coralIntake.stop())
        );
        NamedCommands.registerCommand("IntakeCoral", coralIntake.intake().andThen(coralIntake.stop()));
    }

    /**
     * Returns the selected autonomous command
     *
     * @return A {@link Command} representing the selected autonomous command
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}