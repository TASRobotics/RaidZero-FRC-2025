package raidzero.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
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
import raidzero.robot.Telemetry;

public class AlgaeIntake extends SubsystemBase {
    private static AlgaeIntake system;

    private TalonFX joint;
    private TalonFX climb;
    private SparkMax roller;

    private AlgaeIntake() {
        joint = new TalonFX(Constants.AlgaeIntake.Joint.MOTOR_ID);
        joint.getConfigurator().apply(jointConfiguration());
        joint.setNeutralMode(NeutralModeValue.Brake);

        climb = new TalonFX(Constants.AlgaeIntake.Climb.MOTOR_ID);
        climb.getConfigurator().apply(climbConfiguration());
        climb.setNeutralMode(NeutralModeValue.Brake);

        roller = new SparkMax(Constants.AlgaeIntake.Roller.MOTOR_ID, MotorType.kBrushless);
        roller.configure(rollerConfig(), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public Command runJoint(double setpoint, BooleanSupplier stop_condition) {
        return Commands.run(() -> moveJoint(setpoint), this)
            .until(stop_condition)
            .andThen(() -> stopJoint());
    }

    public Command runRoller(double speed) {
        return Commands.run(() -> moveRoller(speed)).withTimeout(2).andThen(() -> stopRoller());
    }

    private void moveJoint(double setpoint) {
        final MotionMagicVoltage request = new MotionMagicVoltage(0);
        joint.setControl(request.withPosition(setpoint));
    }

    private void stopJoint() {
        joint.stopMotor();
    }

    private void moveRoller(double speed) {
        roller.set(speed);
    }

    private void stopRoller(){
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
            .withKD(Constants.AlgaeIntake.Joint.KD);

        configuration.MotionMagic = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(Constants.AlgaeIntake.Joint.CRUISE_VELOCITY)
            .withMotionMagicAcceleration(Constants.AlgaeIntake.Joint.ACCELERATION)
            .withMotionMagicJerk(Constants.AlgaeIntake.Joint.JERK);

        configuration.HardwareLimitSwitch.ForwardLimitEnable = true;
        configuration.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyClosed;

        configuration.HardwareLimitSwitch.ReverseLimitEnable = true;
        configuration.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyClosed;

        configuration.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        configuration.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0.0;

        return configuration;
    }

    private TalonFXConfiguration climbConfiguration() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        configuration.Slot0 = new Slot0Configs()
            .withKS(Constants.AlgaeIntake.Climb.KS)
            .withKV(Constants.AlgaeIntake.Climb.KV)
            .withKA(Constants.AlgaeIntake.Climb.KA)
            .withKG(Constants.AlgaeIntake.Climb.KG)
            .withKP(Constants.AlgaeIntake.Climb.KP)
            .withKI(Constants.AlgaeIntake.Climb.KI)
            .withKD(Constants.AlgaeIntake.Climb.KD);

        configuration.MotionMagic = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(Constants.AlgaeIntake.Climb.CRUISE_VELOCITY)
            .withMotionMagicAcceleration(Constants.AlgaeIntake.Climb.ACCELERATION)
            .withMotionMagicJerk(Constants.AlgaeIntake.Climb.JERK);

        configuration.HardwareLimitSwitch.ForwardLimitEnable = true;
        configuration.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyClosed;

        configuration.HardwareLimitSwitch.ReverseLimitEnable = true;
        configuration.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyClosed;

        configuration.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        configuration.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0.0;

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