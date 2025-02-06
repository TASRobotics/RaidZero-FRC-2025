// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package raidzero.robot;

import static edu.wpi.first.units.Units.*;

import java.lang.constant.Constable;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import raidzero.robot.subsystems.Swerve;
import raidzero.robot.subsystems.telescopingarm.*;
import raidzero.robot.subsystems.telescopingarm.Constants;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.1; // kSpeedAt12Volts desired top speed
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

    public RobotContainer() {
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

        arm.setDefaultCommand(arm.stopAllCommand());

        joystick.a().whileTrue(swerve.applyRequest(() -> brake));

        joystick.b().whileTrue(arm.moveArm(Constants.L3_SCORING_POS_M[0], Constants.L3_SCORING_POS_M[1]));
        joystick.x().whileTrue(arm.moveArm(Constants.L2_SCORING_POS_M[0], Constants.L2_SCORING_POS_M[1]));
        joystick.y().whileTrue(arm.moveArmWithRotations(0.25, 0.0));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(swerve.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(swerve.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(swerve.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(swerve.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(swerve.runOnce(() -> swerve.seedFieldCentric()));

        swerve.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}