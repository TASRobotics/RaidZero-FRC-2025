// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package raidzero.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import raidzero.robot.subsystems.drivetrain.Limelight;
import raidzero.robot.subsystems.drivetrain.Swerve;
import raidzero.robot.subsystems.drivetrain.TunerConstants;
import raidzero.robot.subsystems.telescopingarm.*;
import raidzero.robot.subsystems.telescopingarm.Constants;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final Swerve swerve = Swerve.system();
    public final Arm arm = Arm.system();
    public final Intake intake = Intake.system();
    public final Limelight limes = Limelight.system();

    public final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("AutoChooser", autoChooser);

        configureBindings();

        // * Set positions for things here in the future
        // arm.resetJointPosition();
        arm.setJointPosition(0.25);
    }

    private void configureBindings() {
        swerve.setDefaultCommand(
            swerve.applyRequest(
                () -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
        );

        arm.setDefaultCommand(arm.moveArmWithRotations(arm.calculateJointAngle(Constants.INTAKE_POS_M[0], Constants.INTAKE_POS_M[1]), 0.0));

        joystick.a().whileTrue(swerve.applyRequest(() -> brake));

        joystick.b().whileTrue(arm.moveArm(Constants.L3_SCORING_POS_M[0], Constants.L3_SCORING_POS_M[1]));
        joystick.x().whileTrue(arm.moveArm(Constants.INTAKE_POS_M[0], Constants.INTAKE_POS_M[1]));
        joystick.a().whileTrue(arm.moveArm(Constants.L4_SCORING_POS_M[0], Constants.L4_SCORING_POS_M[1]));
        joystick.y().whileTrue(arm.moveArmWithRotations(0.25, 0.0));

        joystick.rightTrigger().onTrue(intake.runIntake(0.1));
        joystick.leftTrigger().onTrue(intake.extake(0.1));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(swerve.runOnce(() -> swerve.seedFieldCentric()));

        swerve.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return AutoBuilder.buildAuto("Auton1");
    }
}