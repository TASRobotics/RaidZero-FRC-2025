package raidzero.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidzero.robot.Constants;

public class AlgaeIntake extends SubsystemBase {
    private static AlgaeIntake system;

    private TalonFX joint;
    private SparkMax roller;

    private AlgaeIntake() {
        joint = new TalonFX(Constants.AlgaeIntake.Joint.MOTOR_ID);
        joint.getConfigurator().apply(jointConfiguration());
        joint.setNeutralMode(NeutralModeValue.Brake);

        roller = new SparkMax(Constants.AlgaeIntake.Roller.MOTOR_ID, MotorType.kBrushless);
        roller.configure(rollerConfig(), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public Command runJoint(double setpoint) {
        return Commands.run(() -> {
            moveJoint(setpoint);
            runRoller(Constants.AlgaeIntake.Roller.ROLLER_SPEED);
        }, this)
            .until(() -> jointWithinSetpoint(setpoint) || jointCurrentSpike(Constants.AlgaeIntake.Joint.CURRENT_SPIKE_THRESHOLD_AMPS))
            .andThen(() -> {
                stopJoint();
                stopRoller();
            });
    }

    public Command runRoller(double speed) {
        return Commands.run(() -> moveRoller(speed)).withTimeout(2).andThen(() -> stopRoller());
    }

    private void moveJoint(double setpoint) {
        final PositionTorqueCurrentFOC request = new PositionTorqueCurrentFOC(0);
        joint.setControl(request.withPosition(setpoint));
    }

    private boolean jointCurrentSpike(double currentThreshold) {
        if (joint.getStatorCurrent().getValueAsDouble() > currentThreshold)
            return true;
        return false;
    }

    private boolean jointWithinSetpoint(double setpoint) {
        if (Math.abs(joint.getPosition().getValueAsDouble() - setpoint) < Constants.AlgaeIntake.Joint.POSITION_TOLERANCE)
            return true;
        return false;
    }

    private void stopJoint() {
        joint.stopMotor();
    }

    private void moveRoller(double speed) {
        roller.set(speed);
    }

    private void stopRoller() {
        roller.stopMotor();
    }

    private TalonFXConfiguration jointConfiguration() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        configuration.Slot0 = new Slot0Configs()
            .withKS(Constants.AlgaeIntake.Joint.KS)
            .withKV(Constants.AlgaeIntake.Joint.KV)
            .withKA(Constants.AlgaeIntake.Joint.KA)
            .withKG(Constants.AlgaeIntake.Joint.KG)
            .withKP(Constants.AlgaeIntake.Joint.KP)
            .withKI(Constants.AlgaeIntake.Joint.KI)
            .withKD(Constants.AlgaeIntake.Joint.KD)
            .withGravityType(GravityTypeValue.Arm_Cosine);

        configuration.TorqueCurrent.PeakForwardTorqueCurrent = Constants.AlgaeIntake.Joint.CURRENT_LIMIT;

        configuration.Feedback.SensorToMechanismRatio = Constants.AlgaeIntake.Joint.CONVERSION_FACTOR;

        return configuration;
    }

    private SparkBaseConfig rollerConfig() {
        SparkMaxConfig configuration = new SparkMaxConfig();

        configuration.idleMode(IdleMode.kBrake);

        return configuration;
    }

    public static AlgaeIntake system() {
        if (system == null)
            system = new AlgaeIntake();
        return system;
    }
}