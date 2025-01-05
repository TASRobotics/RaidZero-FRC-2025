package raidzero.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidzero.robot.Constants;

/**
 * Telescoping arm skeleton code
 */
public class TelescopingArm extends SubsystemBase {
    private static TelescopingArm system;


    private TalonFX telescope, arm, wrist;

    private TelescopingArm() {
        telescope = new TalonFX(Constants.TelescopingArm.TELESCOPE_MOTOR_ID);
        telescope.getConfigurator().apply(telescopeConfiguration());
        telescope.setNeutralMode(NeutralModeValue.Brake);

        arm = new TalonFX(Constants.TelescopingArm.ARM_MOTOR_ID);
        arm.getConfigurator().apply((wristConfiguration()));
        arm.setNeutralMode(NeutralModeValue.Brake);

        wrist = new TalonFX(Constants.TelescopingArm.WRIST_MOTOR_ID);
        wrist.getConfigurator().apply(armConfiguration());
        wrist.setNeutralMode(NeutralModeValue.Brake);
    }

    /**
     * Method to automatically calculate the arm's position and move it to where it needs to be
     * 
     * @param x the x setpoint
     * @param y the y setpoint
     * @return if it is done or not 
     */
    public double[] moveArmTo(double x, double y, double wristAngle) {
        final MotionMagicVoltage telescopeRequest = new MotionMagicVoltage(0);
        telescope.setControl(telescopeRequest.withPosition(calculateMinTelescopeHeight(x, y)));

        final MotionMagicVoltage armRequest = new MotionMagicVoltage(0);
        arm.setControl(armRequest.withPosition(calcualteArmAngle(x, y)));

        final MotionMagicVoltage wristRequest = new MotionMagicVoltage(0);
        arm.setControl(wristRequest.withPosition(wristAngle * Constants.TelescopingArm.WRIST_CONVERSION_FACTOR));

        return new double[] {
            calculateMinTelescopeHeight(x, y) / Constants.TelescopingArm.ARM_CONVERSION_FACTOR,
            calcualteArmAngle(x, y) / Constants.TelescopingArm.ARM_CONVERSION_FACTOR,
            wristAngle
        };
    }

    private double calculateMinTelescopeHeight(double x, double y) throws IllegalStateException {
        double height = y - Constants.TelescopingArm.ARM_LENGTH_M * Math.sin(Math.acos(x / Constants.TelescopingArm.ARM_LENGTH_M)) * Constants.TelescopingArm.TELESCOPE_CONVERSION_FACTOR;

        if (height > Constants.TelescopingArm.TELESCOPE_MAX_LENGTH_M)
            throw new IllegalStateException("Arm setpoint too high");

        return height;
    }

    private double calcualteArmAngle(double x, double y) {
        return Math.acos(x / Constants.TelescopingArm.ARM_LENGTH_M) + (Math.PI / 4) * Constants.TelescopingArm.ARM_CONVERSION_FACTOR;
    }

    /**
     * Get the telescope motor's encoder position
     * 
     * @return telescope motor encoder position in rotations
     */
    public double getTelescopePosition() {
        return telescope.getPosition().getValueAsDouble();
    }

    /**
     * Get the arm motor's encoder position
     * 
     * @return arm motor encoder position in rotations
     */
    public double getArmPosition() {
        return arm.getPosition().getValueAsDouble();
    }

    /**
     * Get the wrist motor's encoder position
     * 
     * @return wrist motor encoder position in rotations
     */
    public double getWristPosition() {
        return wrist.getPosition().getValueAsDouble();
    }

    public void stopTelescope() {
        telescope.stopMotor();
    }

    public void stopArm() {
        arm.stopMotor();
    }

    public void stopWrist() {
        wrist.stopMotor();
    }

    public static TelescopingArm system() {
        if (system == null) {
            system = new TelescopingArm();
        }
        return system;
    }

    private TalonFXConfiguration telescopeConfiguration() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        configuration.Slot0 = new Slot0Configs()
            .withKS(Constants.TelescopingArm.TELESCOPE_KS)
            .withKV(Constants.TelescopingArm.TELESCOPE_KV)
            .withKA(Constants.TelescopingArm.TELESCOPE_KA)
            .withKP(Constants.TelescopingArm.TELESCOPE_KP)
            .withKI(Constants.TelescopingArm.TELESCOPE_KI)
            .withKD(Constants.TelescopingArm.TELESCOPE_KD);

        configuration.MotionMagic = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(Constants.TelescopingArm.TELESCOPE_CRUISE_VELOCITY)
            .withMotionMagicAcceleration(Constants.TelescopingArm.TELESCOPE_ACCELERATION)
            .withMotionMagicJerk(Constants.TelescopingArm.TELESCOPE_JERK);

        return configuration;
    }

        private TalonFXConfiguration armConfiguration() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        configuration.Slot0 = new Slot0Configs()
            .withKS(Constants.TelescopingArm.ARM_KS)
            .withKV(Constants.TelescopingArm.ARM_KV)
            .withKA(Constants.TelescopingArm.ARM_KA)
            .withKP(Constants.TelescopingArm.ARM_KP)
            .withKI(Constants.TelescopingArm.ARM_KI)
            .withKD(Constants.TelescopingArm.ARM_KD);

        configuration.MotionMagic = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(Constants.TelescopingArm.ARM_CRUISE_VELOCITY)
            .withMotionMagicAcceleration(Constants.TelescopingArm.ARM_ACCELERATION)
            .withMotionMagicJerk(Constants.TelescopingArm.ARM_JERK);

        return configuration;
        }

        private TalonFXConfiguration wristConfiguration() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        configuration.Slot0 = new Slot0Configs()
            .withKS(Constants.TelescopingArm.WRIST_KS)
            .withKV(Constants.TelescopingArm.WRIST_KV)
            .withKA(Constants.TelescopingArm.WRIST_KA)
            .withKP(Constants.TelescopingArm.WRIST_KP)
            .withKI(Constants.TelescopingArm.WRIST_KI)
            .withKD(Constants.TelescopingArm.WRIST_KD);

        configuration.MotionMagic = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(Constants.TelescopingArm.WRIST_CRUISE_VELOCITY)
            .withMotionMagicAcceleration(Constants.TelescopingArm.WRIST_ACCELERATION)
            .withMotionMagicJerk(Constants.TelescopingArm.WRIST_JERK);

        return configuration;
        }
}