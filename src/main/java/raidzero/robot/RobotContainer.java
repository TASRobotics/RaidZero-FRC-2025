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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import raidzero.robot.subsystems.algaeintake.AlgaeJoint;
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

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandGenericHID operator = new CommandGenericHID(1);

    public final Swerve swerve = Swerve.system();

    public final Arm arm = Arm.system();
    public final CoralIntake coralIntake = CoralIntake.system();

    public final Limelight limes = Limelight.system();

    public final AlgaeJoint algaeIntake = AlgaeJoint.system();

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

        climbJoint.setPosition(0.25);
    }

    /**
     * Configures button bindings for the robot
     */
    private void configureBindings() {
        swerve.setDefaultCommand(
            swerve.applyRequest(
                () -> fieldCentricDrive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
        );

        arm.setDefaultCommand(arm.moveArmWithDelay(Constants.TelescopingArm.Positions.INTAKE_POS_M));
        coralIntake.setDefaultCommand(coralIntake.stopRoller());

        algaeIntake.setDefaultCommand(algaeIntake.moveJoint(0.3));

        climbJoint.setDefaultCommand(climbJoint.moveJoint(0.25));
        climbWinch.setDefaultCommand(climbWinch.stopMotor());

        // * Driver controls
        joystick.a().whileTrue(
            swerve.applyRequest(
                () -> robotCentricDrive.withVelocityY(-joystick.getLeftX() * MaxSpeed * 0.3)
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

        // * Operator controls
        operator.button(Constants.Bindings.L3).whileTrue(arm.moveArm(Constants.TelescopingArm.Positions.L3_SCORING_POS_M));
        operator.button(Constants.Bindings.L4).whileTrue(arm.moveArm(Constants.TelescopingArm.Positions.L4_SCORING_POS_M));

        operator.button(Constants.Bindings.CORAL_EXTAKE).whileTrue(coralIntake.extake());
        operator.button(Constants.Bindings.CORAL_INTAKE).onTrue(coralIntake.intake());
        operator.button(Constants.Bindings.CORAL_SCOOCH).onTrue(coralIntake.scoochCoral());

        operator.button(Constants.Bindings.CLIMB_DEPLOY)
            .onTrue(
                arm.vertical().alongWith(
                    Commands.waitSeconds(0.2)
                        .andThen(
                            climbJoint.moveJoint(0.0).until(() -> operator.button(Constants.Bindings.CLIMB_UP).getAsBoolean())
                                .andThen(() -> climbJoint.stop()).alongWith(new InstantCommand(() -> climbJoint.setDeployedState()))
                        )
                )
            );
        operator.button(Constants.Bindings.CLIMB_UP).whileTrue(climbWinch.runWinch(0.1).onlyIf(climbJoint.isDeployed()));
        operator.button(Constants.Bindings.CLIMB_DOWN).whileTrue(climbWinch.runWinch(-0.1).onlyIf(climbJoint.isDeployed()));

        swerve.registerTelemetry(logger::telemeterize);
    }

    /**
     * Registers PathPlanner commands
     */
    private void registerPathplannerCommands() {
        NamedCommands.registerCommand("ArmIntakeCoral", arm.moveArm(Constants.TelescopingArm.Positions.INTAKE_POS_M));
        NamedCommands.registerCommand("ArmL3", arm.moveArm(Constants.TelescopingArm.Positions.L3_SCORING_POS_M));
        NamedCommands.registerCommand("ArmL4", arm.moveArm(Constants.TelescopingArm.Positions.L4_SCORING_POS_M));

        NamedCommands.registerCommand(
            "ExtakeCoral", coralIntake.extake().until(
                () -> coralIntake.getBottomLaserDistance() >= Constants.TelescopingArm.Intake.LASERCAN_DISTANCE_THRESHOLD_MM
            ).withTimeout(1.0).andThen(() -> coralIntake.stopRoller())
        );
        NamedCommands.registerCommand("IntakeCoral", coralIntake.intake().withTimeout(0.8).andThen(() -> coralIntake.stopRoller()));
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