package raidzero.robot.subsystems.algaedescore;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidzero.robot.Constants;
import raidzero.robot.subsystems.algaeintake.AlgaeJoint;

public class Telescope extends SubsystemBase{
    private TalonFX telescope;
    private static Telescope system;

    private Telescope(){
        telescope = new TalonFX(Constants.AlgaeDescore.Telescope.MOTOR_ID, "rio");
        telescope.getConfigurator().apply(telescopeConfiguration());
        telescope.setNeutralMode(NeutralModeValue.Brake);
    }

    public Command moveTo(double setpoint){
        return run(() -> telescope.setControl(new MotionMagicVoltage(0).withPosition(setpoint)));
    }

    public void stop(){
        telescope.stopMotor();
    }

    private TalonFXConfiguration telescopeConfiguration() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        configuration.Slot0 = new Slot0Configs()
            .withKS(Constants.AlgaeDescore.Telescope.KS)
            .withKV(Constants.AlgaeDescore.Telescope.KV)
            .withKA(Constants.AlgaeDescore.Telescope.KA)
            .withKG(Constants.AlgaeDescore.Telescope.KG)
            .withKP(Constants.AlgaeDescore.Telescope.KP)
            .withKI(Constants.AlgaeDescore.Telescope.KI)
            .withKD(Constants.AlgaeDescore.Telescope.KD);

        configuration.MotionMagic = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(Constants.AlgaeDescore.Telescope.CRUISE_VELOCITY)
            .withMotionMagicAcceleration(Constants.AlgaeDescore.Telescope.ACCELERATION)
            .withMotionMagicJerk(Constants.AlgaeDescore.Telescope.JERK);

        configuration.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        configuration.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0.0;
        configuration.HardwareLimitSwitch.ReverseLimitEnable = false;

        configuration.Feedback.SensorToMechanismRatio = Constants.AlgaeDescore.Telescope.CONVERSION_FACTOR;

        configuration.Slot0.GravityType = Constants.AlgaeDescore.Telescope.GRAVITY_TYPE_VALUE;

        return configuration;
    }

    public static Telescope system() {
        if (system == null) {
            system = new Telescope();
        }

        return system;
    }
}