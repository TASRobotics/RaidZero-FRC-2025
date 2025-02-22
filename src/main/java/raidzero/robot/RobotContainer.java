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
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import raidzero.robot.subsystems.algaeintake.Joint;
import raidzero.robot.subsystems.drivetrain.Limelight;
import raidzero.robot.subsystems.drivetrain.Swerve;
import raidzero.robot.subsystems.drivetrain.TunerConstants;
import raidzero.robot.subsystems.telescopingarm.*;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandGenericHID operator = new CommandGenericHID(1);

    public final Swerve swerve = Swerve.system();
    public final Arm arm = Arm.system();
    public final Intake intake = Intake.system();
    public final Limelight limes = Limelight.system();
    public final Joint algaeIntake = Joint.system();

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
    }

    /**
     * Configures button bindings for the robot
     */
    private void configureBindings() {
        swerve.setDefaultCommand(
            swerve.applyRequest(
                () -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
        );

        arm.setDefaultCommand(arm.moveArm(Constants.TelescopingArm.Positions.INTAKE_POS_M[0], Constants.TelescopingArm.Positions.INTAKE_POS_M[1]));
        intake.setDefaultCommand(intake.stopRollerCommand());
        
        algaeIntake.setDefaultCommand(algaeIntake.moveJoint(0.3));

        //* Driver controls
        joystick.leftBumper().whileTrue(intake.extake(0.1));
        joystick.rightBumper().onTrue(intake.runIntake(0.1));

        joystick.x().whileTrue(
            swerve.pathToReef(Constants.Swerve.REEFS.LEFT)
        );

        joystick.y().whileTrue(
            swerve.pathToReef(Constants.Swerve.REEFS.RIGHT)
        );

        joystick.povLeft().whileTrue(
            swerve.pathToStation()
        );

        //* Operator controls
        operator.button(8).whileTrue(arm.moveArm(Constants.TelescopingArm.Positions.L3_SCORING_POS_M[0], Constants.TelescopingArm.Positions.L3_SCORING_POS_M[1]));
        operator.button(9).whileTrue(arm.moveArm(Constants.TelescopingArm.Positions.L4_SCORING_POS_M[0], Constants.TelescopingArm.Positions.L4_SCORING_POS_M[1]));

        operator.button(10).whileTrue(intake.extake(0.1));
        operator.button(11).onTrue(intake.runIntake(0.1));
        operator.button(12).whileTrue(arm.moveArm(Constants.TelescopingArm.Positions.INTAKE_POS_M[0], Constants.TelescopingArm.Positions.INTAKE_POS_M[1]));

        swerve.registerTelemetry(logger::telemeterize);
    }

    /**
     * Registers PathPlanner commands
     */
    private void registerPathplannerCommands() {
        NamedCommands.registerCommand("ArmIntakeCoral", arm.moveArm(Constants.TelescopingArm.Positions.INTAKE_POS_M[0], Constants.TelescopingArm.Positions.INTAKE_POS_M[1]));
        NamedCommands.registerCommand("ArmL3", arm.moveArm(Constants.TelescopingArm.Positions.L3_SCORING_POS_M[0], Constants.TelescopingArm.Positions.L3_SCORING_POS_M[1]));

        NamedCommands.registerCommand(
            "ExtakeCoral", intake.extake(0.15).until(
                () -> intake.getLimitDistance() >= 40
            ).withTimeout(1.0).andThen(() -> intake.stopRoller())
        );
        NamedCommands.registerCommand("IntakeCoral", intake.runIntake(0.12).withTimeout(0.8).andThen(() -> intake.stopRoller()));
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