package raidzero.robot.subsystems.algaeintake;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import raidzero.robot.subsystems.algaeintake.Constants.JointC;

public class Joint extends SubsystemBase {
    private static Joint system;

    private TalonFX joint;

    private Joint() {
        joint = new TalonFX(JointC.MOTOR_ID);
        joint.getConfigurator().apply(jointConfiguration());
        joint.setNeutralMode(NeutralModeValue.Brake);
    }

    private void moveJoint(double setpoint) {
        final PositionVoltage request = new PositionVoltage(0);
        joint.setControl(request.withPosition(setpoint));
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

        configuration.Feedback.SensorToMechanismRatio = JointC.CONVERSION_FACTOR;

        configuration.Slot0 = new Slot0Configs()
            .withKS(JointC.KS)  //inc until moves
            .withKV(JointC.KV) //min volt required to get to one rotation per sec w/ conversion factor
            .withKA(JointC.KA) //achieve smooth acceleration w/o overshoot
            .withKG(JointC.KG) //to move up
            .withKP(JointC.KP)
            .withKI(JointC.KI)
            .withKD(JointC.KD)
            .withGravityType(GravityTypeValue.Arm_Cosine);

        configuration.Slot0.GravityType = JointC.GRAVITY_TYPE;

        configuration.CurrentLimits.StatorCurrentLimit = JointC.CURRENT_LIMIT;
        configuration.CurrentLimits.SupplyCurrentLimit = JointC.SUPPLY_CURRENT_LIMIT;
        configuration.CurrentLimits.SupplyCurrentLowerTime = JointC.SUPPLY_CURRENT_LOWER_TIME;

        configuration.MotionMagic.MotionMagicCruiseVelocity = JointC.MOTION_MAGIC_CRUISE_VELOCITY;
        configuration.MotionMagic.MotionMagicAcceleration = JointC.MOTION_MAGIC_ACCELERATION;

        return configuration;
    }

    public static Joint system() {
        if (system == null)
            system = new Joint();
        return system;
    }
}