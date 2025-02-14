package raidzero.robot.subsystems.climb;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import raidzero.robot.subsystems.climb.Constants.Joint;
import raidzero.robot.subsystems.climb.Constants.Winch;

public class Climb extends SubsystemBase {
    private static Climb system;

    private TalonFX joint;
    private TalonFX winch;

    private Climb() {
        joint = new TalonFX(Joint.MOTOR_ID);
        joint.getConfigurator().apply(jointConfig());

        winch = new TalonFX(Winch.MOTOR_ID);
        winch.getConfigurator().apply(winchConfiguration());
        winch.setNeutralMode(NeutralModeValue.Brake);
    }

    public Command runJoint(double setpoint, BooleanSupplier stopCondition) {
        final MotionMagicVoltage request = new MotionMagicVoltage(0);
        return Commands.run(() -> {
            joint.setControl(request.withPosition(setpoint));
        }, this).until(stopCondition);
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

    private TalonFXConfiguration jointConfig() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        configuration.Feedback.SensorToMechanismRatio = Joint.CONVERSION_FACTOR;

        configuration.Slot0 = new Slot0Configs()
            .withKS(Joint.KS)
            .withKV(Joint.KV)
            .withKA(Joint.KA)
            .withKG(Joint.KG)
            .withKP(Joint.KP)
            .withKI(Joint.KI)
            .withKD(Joint.KD);

        configuration.Slot0.GravityType = Joint.GRAVITY_TYPE_VALUE;

        configuration.MotionMagic = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(Joint.CRUISE_VELOCITY)
            .withMotionMagicAcceleration(Joint.ACCELERATION)
            .withMotionMagicJerk(Joint.JERK);

        configuration.CurrentLimits.StatorCurrentLimit = Joint.STATOR_CURRENT_LIMT;
        configuration.CurrentLimits.SupplyCurrentLimit = Joint.SUPPLY_CURRENT_LIMIT;
        configuration.CurrentLimits.SupplyCurrentLowerTime = Joint.SUPPLY_CURRENT_LOWER_TIME;

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
