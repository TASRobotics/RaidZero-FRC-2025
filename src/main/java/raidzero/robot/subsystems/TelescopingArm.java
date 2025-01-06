package raidzero.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidzero.robot.Constants;

public class TelescopingArm extends SubsystemBase {
    private static TelescopingArm system;

    private TalonFX telescope, armJoint;

    /**
     * Constructor for the {@link TelescopingArm} subsystem
     */
    private TelescopingArm() {
        telescope = new TalonFX(Constants.TelescopingArm.Telescope.MOTOR_ID);
        telescope.getConfigurator().apply(telescopeConfiguration());
        telescope.setNeutralMode(NeutralModeValue.Brake);

        armJoint = new TalonFX(Constants.TelescopingArm.ArmJoint.MOTOR_ID);
        armJoint.getConfigurator().apply((armConfiguration()));
        armJoint.setNeutralMode(NeutralModeValue.Brake);
    }

    /**
     * Method to calculate the arm's position and move it to the desired setpoint
     * 
     * @param x the x setpoint in meters
     * @param y the y setpoint in meters
     * @return the height and angle setpoints in rotations for the telescope and arm respectively 
     */
    public double[] moveArmTo(double x, double y) {
        final MotionMagicVoltage telescopeRequest = new MotionMagicVoltage(0);
        telescope.setControl(telescopeRequest.withPosition(calculateTelescopeHeight(x, y)));

        final MotionMagicVoltage armRequest = new MotionMagicVoltage(0);
        armJoint.setControl(armRequest.withPosition(calculateArmAngle(x, y)));

        return new double[] {
                calculateTelescopeHeight(x, y),
                calculateArmAngle(x, y)
        };
    }

    /**
     * Calculates the target telescope position given the x and y setpoints
     * 
     * @param x the x setpoint in meters
     * @param y the y setpoint in meters
     * @return target motor position in rotations
     * @throws IllegalStateException if the calculated target height is higher than the maximum height
     */
    private double calculateTelescopeHeight(double x, double y) throws IllegalStateException {
        double height = Math.sqrt(x * x + y * y);

        if (height > Constants.TelescopingArm.Telescope.MAX_LENGTH_M)
            throw new IllegalStateException("Arm setpoint too high");

        return height * Constants.TelescopingArm.Telescope.CONVERSION_FACTOR;
    }

    /**
     * Calculates the target arm position given the x and y setpoints
     * 
     * @param x the x setpoint in meters
     * @param y the y setpoint in meters
     * @return the target arm position in rotations
     */
    private double calculateArmAngle(double x, double y) {
        return Math.atan2(y, x) * Constants.TelescopingArm.ArmJoint.CONVERSION_FACTOR;
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
        return armJoint.getPosition().getValueAsDouble();
    }

    /**
     * Stop the telescope motor
     */
    public void stopTelescope() {
        telescope.stopMotor();
    }

    /**
     * Stop the arm motor
     */
    public void stopArm() {
        armJoint.stopMotor();
    }

    /**
     * Stop all motors
     */
    public void stopAll() {
        stopTelescope();
        stopArm();
    }

    /**
     * Get the singleton instance of the {@link TelescopingArm} subsystem
     * 
     * @return the {@link TelescopingArm} subsystem
     */
    public static TelescopingArm system() {
        if (system == null) {
            system = new TelescopingArm();
        }

        return system;
    }

    /**
     * Get the {@link TalonFXConfiguration} for the telescope
     * 
     * @return the {@link TalonFXConfiguration} for the telescope
     */
    private TalonFXConfiguration telescopeConfiguration() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        configuration.Slot0 = new Slot0Configs()
                .withKS(Constants.TelescopingArm.Telescope.KS)
                .withKV(Constants.TelescopingArm.Telescope.KV)
                .withKA(Constants.TelescopingArm.Telescope.KA)
                .withKP(Constants.TelescopingArm.Telescope.KP)
                .withKI(Constants.TelescopingArm.Telescope.KI)
                .withKD(Constants.TelescopingArm.Telescope.KD);

        configuration.MotionMagic = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(Constants.TelescopingArm.Telescope.CRUISE_VELOCITY)
                .withMotionMagicAcceleration(Constants.TelescopingArm.Telescope.ACCELERATION)
                .withMotionMagicJerk(Constants.TelescopingArm.Telescope.JERK);

        return configuration;
    }

    /**
     * Get the {@link TalonFXConfiguration} for the arm joint
     * 
     * @return the {@link TalonFXConfiguration} for the arm joint
     */
    private TalonFXConfiguration armConfiguration() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        configuration.Slot0 = new Slot0Configs()
                .withKS(Constants.TelescopingArm.ArmJoint.KS)
                .withKV(Constants.TelescopingArm.ArmJoint.KV)
                .withKA(Constants.TelescopingArm.ArmJoint.KA)
                .withKP(Constants.TelescopingArm.ArmJoint.KP)
                .withKI(Constants.TelescopingArm.ArmJoint.KI)
                .withKD(Constants.TelescopingArm.ArmJoint.KD);

        configuration.MotionMagic = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(Constants.TelescopingArm.ArmJoint.CRUISE_VELOCITY)
                .withMotionMagicAcceleration(Constants.TelescopingArm.ArmJoint.ACCELERATION)
                .withMotionMagicJerk(Constants.TelescopingArm.ArmJoint.JERK);

        return configuration;
    }
}