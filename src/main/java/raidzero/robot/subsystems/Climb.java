package raidzero.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidzero.robot.Constants;

public class Climb extends SubsystemBase {
    private static Climb system;

    private SparkMax joint;
    private SparkClosedLoopController jointController;

    private TalonFX winch;

    private Climb() {
        joint = new SparkMax(Constants.Climb.Joint.MOTOR_ID, MotorType.kBrushless);
        joint.configure(jointConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        jointController = joint.getClosedLoopController();

        winch = new TalonFX(Constants.Climb.Winch.MOTOR_ID);
        winch.getConfigurator().apply(winchConfiguration());
        winch.setNeutralMode(NeutralModeValue.Brake);
    }

    public Command runJoint(double setpoint, BooleanSupplier stopCondition) {
        return Commands.run(() -> jointController.setReference(setpoint, ControlType.kPosition), this)
            .until(stopCondition)
            .andThen(() -> joint.stopMotor());
    }

    public Command runWinch() {
        return Commands.run(() -> winch.set(0.3));
    }

    public Command stopClimb() {
        return Commands.run(() -> winch.stopMotor(), this);
    }

    public Command unwindWinch() {
        return Commands.run(() -> winch.set(-0.1), this);
    }

    // private boolean jointWithinSetpoint(double setpoint) {
    // if (Math.abs(setpoint - joint.getEncoder().getPosition()) < 0.2)
    // return true;
    // return false;
    // }

    private SparkBaseConfig jointConfig() {
        SparkMaxConfig configuration = new SparkMaxConfig();

        configuration.idleMode(IdleMode.kBrake);
        configuration.closedLoop
            .pidf(Constants.Climb.Joint.KP, Constants.Climb.Joint.KI, Constants.Climb.Joint.KD, Constants.Climb.Joint.KF);
        configuration.smartCurrentLimit(Constants.Climb.Joint.CURRENT_LIMIT);

        return configuration;
    }

    private SparkBaseConfig disableJointBrake() {
        SparkMaxConfig configuration = new SparkMaxConfig();

        configuration.idleMode(IdleMode.kCoast);
        configuration.closedLoop
            .pidf(Constants.Climb.Joint.KP, Constants.Climb.Joint.KI, Constants.Climb.Joint.KD, Constants.Climb.Joint.KF);
        configuration.smartCurrentLimit(Constants.Climb.Joint.CURRENT_LIMIT);

        return configuration;
    }

    private TalonFXConfiguration winchConfiguration() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        return configuration;
    }

    public static Climb system() {
        if (system == null) {
            system = new Climb();
        }
        return system;
    }
}
