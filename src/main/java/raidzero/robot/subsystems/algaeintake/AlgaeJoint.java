package raidzero.robot.subsystems.algaeintake;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidzero.robot.Constants;

public class AlgaeJoint extends SubsystemBase {
    private TalonFX joint;

    private static AlgaeJoint system;

    /**
     * Constructs a {@link AlgaeJoint} subsystem instance
     */
    private AlgaeJoint() {
        joint = new TalonFX(Constants.AlgaeIntake.Joint.MOTOR_ID);
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
        return run(() -> joint.setControl(new MotionMagicVoltage(0).withPosition(setpoint)));
    }

    /**
     * Stops the joint motor
     */
    public void stop() {
        joint.stopMotor();
    }

    /**
     * Gets the {@link TalonFXConfiguration} for the joint motor
     *
     * @return The {@link TalonFXConfiguration} for the joint motor
     */
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

    /**
     * Gets the {@link AlgaeJoint} subsystem instance
     *
     * @return The {@link AlgaeJoint} subsystem instance
     */
    public static AlgaeJoint system() {
        if (system == null) {
            system = new AlgaeJoint();
        }

        return system;
    }
}