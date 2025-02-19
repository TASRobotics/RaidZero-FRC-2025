package raidzero.robot.subsystems.algaeintake;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import raidzero.robot.Constants;

public class Joint extends SubsystemBase {
    private static Joint system;

    private TalonFX joint;

    private Joint() {
        joint = new TalonFX(Constants.AlgaeIntake.Joint.MOTOR_ID);
        joint.getConfigurator().apply(jointConfiguration());
        joint.setNeutralMode(NeutralModeValue.Brake);
    }

    public Command moveJoint(double setpoint) {
        final MotionMagicVoltage request = new MotionMagicVoltage(0);
        return Commands.run(() -> joint.setControl(request.withPosition(setpoint)), this);
    }

    private boolean jointCurrentSpike(double currentThreshold) {
        if (joint.getStatorCurrent().getValueAsDouble() > currentThreshold)
            return true;
        return false;
    }

    private void stopJoint() {
        joint.stopMotor();
    }

    private TalonFXConfiguration jointConfiguration() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        configuration.Feedback.SensorToMechanismRatio = Constants.AlgaeIntake.Joint.CONVERSION_FACTOR;

        configuration.Slot0 = new Slot0Configs()
            .withKS(Constants.AlgaeIntake.Joint.KS) // inc until moves
            .withKV(Constants.AlgaeIntake.Joint.KV) // min volt required to get to one rotation per sec w/ conversion factor
            .withKA(Constants.AlgaeIntake.Joint.KA) // achieve smooth acceleration w/o overshoot
            .withKG(Constants.AlgaeIntake.Joint.KG) // to move up
            .withKP(Constants.AlgaeIntake.Joint.KP)
            .withKI(Constants.AlgaeIntake.Joint.KI)
            .withKD(Constants.AlgaeIntake.Joint.KD)
            .withGravityType(GravityTypeValue.Arm_Cosine);

        configuration.Slot0.GravityType = Constants.AlgaeIntake.Joint.GRAVITY_TYPE;

        configuration.CurrentLimits.StatorCurrentLimit = Constants.AlgaeIntake.Joint.CURRENT_LIMIT;
        configuration.CurrentLimits.SupplyCurrentLimit = Constants.AlgaeIntake.Joint.SUPPLY_CURRENT_LIMIT;
        configuration.CurrentLimits.SupplyCurrentLowerTime = Constants.AlgaeIntake.Joint.SUPPLY_CURRENT_LOWER_TIME;

        configuration.MotionMagic.MotionMagicCruiseVelocity = Constants.AlgaeIntake.Joint.MOTION_MAGIC_CRUISE_VELOCITY;
        configuration.MotionMagic.MotionMagicAcceleration = Constants.AlgaeIntake.Joint.MOTION_MAGIC_ACCELERATION;

        return configuration;
    }

    public static Joint system() {
        if (system == null)
            system = new Joint();
        return system;
    }
}