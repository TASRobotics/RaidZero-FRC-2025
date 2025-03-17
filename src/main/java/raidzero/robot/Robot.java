// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package raidzero.robot;

import au.grapplerobotics.CanBridge;
import com.ctre.phoenix6.controls.StaticBrake;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import raidzero.lib.Elastic;
import raidzero.robot.subsystems.LEDStrip.ArmStrip;
import raidzero.robot.subsystems.drivetrain.Swerve;
import raidzero.robot.subsystems.telescopingarm.Arm;
import raidzero.robot.subsystems.telescopingarm.CoralIntake;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    public Robot() {
        m_robotContainer = new RobotContainer();
        CanBridge.runTCP();

        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
        Elastic.selectTab("Setup");
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {
        Arm.system().updateCoastMode();

        Swerve.system().initializeOtf();
    }

    @Override
    public void disabledExit() {
        CoralIntake.system().enableStaticBrake();;
        ArmStrip.system().resetAnimation();
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }

        Elastic.selectTab("Autonomous");
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {
        ArmStrip.system().resetAnimation();
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        Elastic.selectTab("Teleoperated");
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {
        ArmStrip.system().resetAnimation();
        ArmStrip.system().matchEndAnimation();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {
        ArmStrip.system().resetAnimation();
    }

    @Override
    public void simulationPeriodic() {}
}
