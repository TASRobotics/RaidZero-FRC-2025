package raidzero.robot.subsystems.algaeintake;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Joint extends SubsystemBase {
    private static Joint system;

    private TalonFX joint;

    private Joint() {
        joint = new TalonFX(Constants.Joint.MOTOR_ID);
        joint.getConfigurator().apply(jointConfiguration());
        joint.setNeutralMode(NeutralModeValue.Brake);
    }

    public Command runJointWithCurrentSpike(double setpoint) {
        return Commands.run(() -> {
            moveJoint(setpoint);
        }, this)
            .until(() -> jointWithinSetpoint(setpoint) || jointCurrentSpike(Constants.Joint.CURRENT_SPIKE_THRESHOLD_AMPS))
            .andThen(() -> {
                stopJoint();
            });
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

    private boolean jointWithinSetpoint(double setpoint) {
        if (Math.abs(joint.getPosition().getValueAsDouble() - setpoint) < Constants.Joint.POSITION_TOLERANCE)
            return true;
        return false;
    }

    private void stopJoint() {
        joint.stopMotor();
    }

    private TalonFXConfiguration jointConfiguration() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        configuration.Slot0 = new Slot0Configs()
            .withKS(Constants.Joint.KS)  //inc until moves
            .withKV(Constants.Joint.KV) //min volt required to get to one rotation per sec w/ conversion factor
            .withKA(Constants.Joint.KA) //achieve smooth acceleration w/o overshoot
            .withKG(Constants.Joint.KG) //to move up
            .withKP(Constants.Joint.KP)
            .withKI(Constants.Joint.KI)
            .withKD(Constants.Joint.KD)
            .withGravityType(GravityTypeValue.Arm_Cosine);

        configuration.TorqueCurrent.PeakForwardTorqueCurrent = Constants.Joint.CURRENT_LIMIT;

        configuration.Feedback.SensorToMechanismRatio = Constants.Joint.CONVERSION_FACTOR;

        return configuration;
    }

    public static Joint system() {
        if (system == null)
            system = new Joint();
        return system;
    }
}