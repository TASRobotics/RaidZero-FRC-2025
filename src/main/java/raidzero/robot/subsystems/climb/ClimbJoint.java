package raidzero.robot.subsystems.climb;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import raidzero.robot.Constants;

public class ClimbJoint extends SubsystemBase {
    private static ClimbJoint system;

    private TalonFX joint;

    boolean isDeployed = false;

    /**
     * Constructs a {@link ClimbJoint} subsystem instance
     */
    private ClimbJoint() {
        joint = new TalonFX(Constants.Climb.Joint.MOTOR_ID);
        joint.getConfigurator().apply(jointConfiguration());
        joint.setNeutralMode(NeutralModeValue.Brake);
    }

    /**
     * Moves the joint to the desired setpoint
     * 
     * @param setpoint The desired setpoint
     * @return A {@link Command} that moves the joint to the desired setpoint
     */
    public Command moveJoint(double setpoint) {
        return run(() -> joint.setControl((new MotionMagicVoltage(0)).withPosition(setpoint)));
    }

    /**
     * Stops the joint motor
     */
    public void stop() {
        joint.stopMotor();
    }

    /**
     * Sets the position of the joint
     * 
     * @param setptiont The desired position of the joint
     */
    public void setPosition(double setptiont) {
        joint.setPosition(setptiont);
    }

    /**
     * Returns a {@link BooleanSupplier} that checks if the joint is deployed
     * 
     * @return A {@link BooleanSupplier} that checks if the joint is deployed
     */
    public BooleanSupplier isDeployed() {
        return () -> isDeployed;
    }

    /**
     * Sets the joint to the deployed state to true
     */
    public void setDeployedState() {
        isDeployed = true;
    }

    /**
     * Gets the {@link TalonFXConfiguration} for the joint motor
     * 
     * @return The {@link TalonFXConfiguration} for the joint motor
     */
    private TalonFXConfiguration jointConfiguration() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        configuration.Feedback.SensorToMechanismRatio = Constants.Climb.Joint.SENSOR_TO_MECHANISM_RATIO;

        configuration.Slot0 = new Slot0Configs()
            .withKS(Constants.Climb.Joint.KS)
            .withKV(Constants.Climb.Joint.KV)
            .withKA(Constants.Climb.Joint.KA)
            .withKG(Constants.Climb.Joint.KG)
            .withKP(Constants.Climb.Joint.KP)
            .withKI(Constants.Climb.Joint.KI)
            .withKD(Constants.Climb.Joint.KD)
            .withGravityType(GravityTypeValue.Arm_Cosine);

        configuration.CurrentLimits.StatorCurrentLimit = Constants.Climb.Joint.CURRENT_LIMIT;
        configuration.CurrentLimits.SupplyCurrentLimit = Constants.Climb.Joint.SUPPLY_CURRENT_LIMIT;
        configuration.CurrentLimits.SupplyCurrentLowerTime = Constants.Climb.Joint.SUPPLY_CURRENT_LOWER_TIME;

        configuration.MotionMagic.MotionMagicCruiseVelocity = Constants.Climb.Joint.MOTION_MAGIC_CRUISE_VELOCITY;
        configuration.MotionMagic.MotionMagicAcceleration = Constants.Climb.Joint.MOTION_MAGIC_ACCELERATION;

        configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Climb.Joint.FORWARD_SOFT_LIMIT;

        configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Climb.Joint.REVERSE_SOFT_LIMIT;

        return configuration;
    }

    /**
     * Gets the {@link ClimbJoint} subsystem instance
     * 
     * @return The {@link ClimbJoint} subsystem instance
     */
    public static ClimbJoint system() {
        if (system == null) {
            system = new ClimbJoint();
        }

        return system;
    }
}
