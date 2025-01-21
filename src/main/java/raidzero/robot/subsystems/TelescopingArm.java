package raidzero.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidzero.robot.Constants;

public class TelescopingArm extends SubsystemBase {
    private static TelescopingArm system;  

    private TalonFX telescope, armJoint;
    private SparkMax roller;

    /**
     * Constructor for the {@link TelescopingArm} subsystem
     */
    private TelescopingArm() {
        telescope = new TalonFX(Constants.TelescopingArm.Telescope.MOTOR_ID);
        telescope.getConfigurator().apply(telescopeConfiguration());
        telescope.setNeutralMode(NeutralModeValue.Coast);

        armJoint = new TalonFX(Constants.TelescopingArm.ArmJoint.MOTOR_ID);
        armJoint.getConfigurator().apply((armConfiguration()));
        armJoint.setNeutralMode(NeutralModeValue.Brake);

        roller = new SparkMax(Constants.TelescopingArm.Roller.MOTOR_ID, MotorType.kBrushless);
        roller.configure(rollerConfiguration(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Creates a {@link Command} to move the arm to the specified x and y setpoints
     * 
     * @param x the x setpoint in meters
     * @param y the y setpoint in meters
     * @return the command to be scheudled and run
     */
    public Command moveArm(double x, double y) {
        double telescopeSetpoint = calculateTelescopeHeight(x, y);
        double armJointSetpoint = calculateArmAngle(x, y);

        return Commands.run(() -> moveTo(telescopeSetpoint, armJointSetpoint), this)
            .until(() -> armWithinSetpoint(telescopeSetpoint, armJointSetpoint))
            .andThen(() -> stopAll());
    }

    /**
     * Zeroes the the relative encoder position in the telescope motor
     * 
     * @return the command to zero the position
     */
    public Command zeroTelescopePosition() {
        return new InstantCommand(() -> telescope.setPosition(0));
    }

    /**
     * Creates a {@link Command} to run the intake at the specified speed
     * 
     * @param speed the speed as a percentage
     * @return the command to be scheduled and run
     */
    public Command intake(double speed) {
        return Commands.run(() -> runRoller(speed), this)
            .withTimeout(2)
            .andThen(() -> stopRoller());
    }

    /**
     * Uses Motion Magic to move the telescope and arm joint to the target position
     * 
     * @param telescopeSetpoint the target telescope position in rotations
     * @param armJointAngle     the target arm joint angle in rotations
     */
    private void moveTo(double telescopeSetpoint, double armJointAngle) {
        final MotionMagicVoltage telescopeRequest = new MotionMagicVoltage(0);
        telescope.setControl(telescopeRequest.withPosition(telescopeSetpoint));

        final MotionMagicVoltage armRequest = new MotionMagicVoltage(0);
        armJoint.setControl(armRequest.withPosition(armJointAngle));
    }

    /**
     * Runs the roller at the specified speed
     * 
     * @param speed the speed to run at as a percentage
     */
    private void runRoller(double speed) {
        roller.set(speed);
    }

    /**
     * Stops the roller motor
     */
    private void stopRoller() {
        roller.stopMotor();
    }

    /**
     * Calculates the target telescope position given the x and y setpoints
     * 
     * @param x the x setpoint in meters
     * @param y the y setpoint in meters
     * @return target motor position in rotations
     * @throws IllegalStateException if the calculated target height is higher than
     *                               the maximum height or lower than the minimum
     *                               height
     */
    private double calculateTelescopeHeight(double x, double y) throws IllegalStateException {
        double height = Math.sqrt(x * x + y * y);

        return height / 360.0;
    }

    /**
     * Calculates the target arm position given the x and y setpoints
     * 
     * @param x the x setpoint in meters
     * @param y the y setpoint in meters
     * @return the target arm position in rotations
     */
    private double calculateArmAngle(double x, double y) {
        return Math.atan2(y, x) / 360.0;
    }

    /**
     * Checks if the arm encoder position is within the defined tolerance in
     * {@link Constants.TelescopingArm.Telescope}
     * 
     * @param telescopeSetpoint the desired telescope position in rotations
     * @param armJointSetpoint  the desired arm joint angle in rotations
     * @return true if both the telescope and the arm are within the defined
     *         tolerance of their setpoints
     */
    private boolean armWithinSetpoint(double telescopeSetpoint, double armJointSetpoint) {
        return ((getTelescopePosition() > telescopeSetpoint - Constants.TelescopingArm.Telescope.POSITION_TOLERANCE_ROTATIONS) &&
            (getTelescopePosition() < telescopeSetpoint + Constants.TelescopingArm.Telescope.POSITION_TOLERANCE_ROTATIONS)) &&
            (getArmPosition() > armJointSetpoint - Constants.TelescopingArm.ArmJoint.POSITION_TOLERANCE_ROTATIONS) &&
            (getArmPosition() < armJointSetpoint + Constants.TelescopingArm.ArmJoint.POSITION_TOLERANCE_ROTATIONS);
    }

    /**
     * Gets the telescope motor's encoder position
     * 
     * @return telescope motor encoder position in rotations
     */
    public double getTelescopePosition() {
        return telescope.getPosition().getValueAsDouble();
    }

    /**
     * Gets the arm motor's encoder position
     * 
     * @return arm motor encoder position in rotations
     */
    public double getArmPosition() {
        return armJoint.getPosition().getValueAsDouble();
    }

    /**
     * Stops the telescope motor
     */
    public void stopTelescope() {
        telescope.stopMotor();
    }

    /**
     * Stops the arm motor
     */
    public void stopArm() {
        armJoint.stopMotor();
    }

    /**
     * Stops all motors
     */
    public void stopAll() {
        stopTelescope();
        stopArm();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator pos", getTelescopePosition());
    }

    /**
     * Gets the {@link TalonFXConfiguration} for the telescope
     * 
     * @return the {@link TalonFXConfiguration} for the telescope
     */
    private TalonFXConfiguration telescopeConfiguration() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        configuration.Slot0 = new Slot0Configs()
            .withKS(Constants.TelescopingArm.Telescope.KS)
            .withKV(Constants.TelescopingArm.Telescope.KV)
            .withKA(Constants.TelescopingArm.Telescope.KA)
            .withKG(Constants.TelescopingArm.Telescope.KG)
            .withKP(Constants.TelescopingArm.Telescope.KP)
            .withKI(Constants.TelescopingArm.Telescope.KI)
            .withKD(Constants.TelescopingArm.Telescope.KD);

        configuration.MotionMagic = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(Constants.TelescopingArm.Telescope.CRUISE_VELOCITY)
            .withMotionMagicAcceleration(Constants.TelescopingArm.Telescope.ACCELERATION)
            .withMotionMagicJerk(Constants.TelescopingArm.Telescope.JERK);

        configuration.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;
        configuration.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = 0.0;

        configuration.Feedback.SensorToMechanismRatio = Constants.TelescopingArm.Telescope.CONVERSION_FACTOR;

        configuration.Slot0.GravityType = Constants.TelescopingArm.Telescope.GRAVITY_TYPE_VALUE;

        return configuration;
    }

    /**
     * Gets the {@link TalonFXConfiguration} for the arm joint
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

    private SparkBaseConfig rollerConfiguration() {
        SparkMaxConfig config = new SparkMaxConfig();

        config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.limitSwitch.forwardLimitSwitchEnabled(true);

        return config;
    }

    /**
     * Gets the singleton instance of the {@link TelescopingArm} subsystem
     * 
     * @return the {@link TelescopingArm} subsystem
     */
    public static TelescopingArm system() {
        if (system == null) {
            system = new TelescopingArm();
        }

        return system;
    }
}