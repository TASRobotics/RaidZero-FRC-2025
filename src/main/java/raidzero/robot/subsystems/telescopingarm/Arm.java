package raidzero.robot.subsystems.telescopingarm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import raidzero.robot.subsystems.telescopingarm.Constants.Telescope;
import raidzero.robot.subsystems.telescopingarm.Constants.Joint;

public class Arm extends SubsystemBase {
    private static Arm system;

    private TalonFX telescope, joint;
    private CANcoder jointCANcoder;

    /**
     * Constructor for the {@link Arm} subsystem
     */
    private Arm() {
        telescope = new TalonFX(Telescope.MOTOR_ID);
        telescope.getConfigurator().apply(telescopeConfiguration());
        telescope.setNeutralMode(NeutralModeValue.Brake);

        joint = new TalonFX(Joint.MOTOR_ID);
        joint.getConfigurator().apply((jointConfiguration()));
        joint.setNeutralMode(NeutralModeValue.Brake);

        jointCANcoder = new CANcoder(Joint.CANCODER_ID);
        jointCANcoder.getConfigurator().apply(jointCANCoderConfiguration());

        joint.setPosition(jointCANcoder.getPosition().getValueAsDouble() / 90.0);
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
        double jointSetpoint = calculateJointAngle(x, y);

        return Commands.run(() -> moveTo(telescopeSetpoint, jointSetpoint), this)
            .until(() -> armWithinSetpoint(telescopeSetpoint, jointSetpoint))
            .andThen(() -> stopAll());
    }

    /**
     * Runs just the telescope to the supplied setpoint
     * 
     * @param setpoint the target setpoint in percentage of full range of motion
     * @return the command to be scheduled and run
     */
    public Command runTelescope(double setpoint) {
        return Commands.run(() -> moveTelescope(setpoint), this);
    }

    public Command runJoint(double setpoint) {
        return Commands.run(() -> moveJoint(setpoint), this);
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
     * Uses Motion Magic to move the telescope and arm joint to the target position
     * 
     * @param telescopeSetpoint the target telescope position in rotations
     * @param armJointAngle     the target arm joint angle in rotations
     */
    private void moveTo(double telescopeSetpoint, double armJointAngle) {
        final MotionMagicVoltage telescopeRequest = new MotionMagicVoltage(0);
        telescope.setControl(telescopeRequest.withPosition(telescopeSetpoint));

        final MotionMagicVoltage armRequest = new MotionMagicVoltage(0);
        joint.setControl(armRequest.withPosition(armJointAngle));
    }

    private void moveTelescope(double setpoint) {
        final MotionMagicVoltage request = new MotionMagicVoltage(0);
        telescope.setControl(request.withPosition(setpoint));
    }

    private void moveJoint(double setpoint) {
        final MotionMagicVoltage request = new MotionMagicVoltage(0);
        joint.setControl(request.withPosition(setpoint));
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

        return height / Telescope.MAX_HEIGHT_M;
    }

    /**
     * Calculates the target arm position given the x and y setpoints
     * 
     * @param x the x setpoint in meters
     * @param y the y setpoint in meters
     * @return the target arm position in rotations
     */
    private double calculateJointAngle(double x, double y) {
        return (Math.atan2(y, x)) * 180 / Math.PI / 360.0;
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
        return Math.abs(getTelescopePosition() - telescopeSetpoint) < Telescope.POSITION_TOLERANCE_ROTATIONS &&
            Math.abs(getJointPosition() - armJointSetpoint) < Joint.POSITION_TOLERANCE_ROTATIONS;
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
    public double getJointPosition() {
        return joint.getPosition().getValueAsDouble();
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
        joint.stopMotor();
    }

    /**
     * Stops all motors
     */
    public void stopAll() {
        stopTelescope();
        stopArm();
    }

    /**
     * Runs periodically in the subsystem
     */
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
            .withKS(Telescope.KS)
            .withKV(Telescope.KV)
            .withKA(Telescope.KA)
            .withKG(Telescope.KG)
            .withKP(Telescope.KP)
            .withKI(Telescope.KI)
            .withKD(Telescope.KD);

        configuration.MotionMagic = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(Telescope.CRUISE_VELOCITY)
            .withMotionMagicAcceleration(Telescope.ACCELERATION)
            .withMotionMagicJerk(Telescope.JERK);

        configuration.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;
        configuration.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = 0.0;

        configuration.Feedback.SensorToMechanismRatio = Telescope.CONVERSION_FACTOR;

        configuration.Slot0.GravityType = Telescope.GRAVITY_TYPE_VALUE;

        return configuration;
    }

    /**
     * Gets the {@link TalonFXConfiguration} for the arm joint
     * 
     * @return the {@link TalonFXConfiguration} for the arm joint
     */
    private TalonFXConfiguration jointConfiguration() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        configuration.Feedback.SensorToMechanismRatio = Joint.CONVERSION_FACTOR;

        configuration.Slot0 = new Slot0Configs()
            .withKS(Joint.KS)
            .withKV(Joint.KV)
            .withKA(Joint.KA)
            .withKG(Joint.KG)
            .withKP(Joint.KP)
            .withKI(Joint.KI)
            .withKD(Joint.KD);

        configuration.Slot0.GravityType = Joint.GRAVITY_TYPE_VALUE;

        configuration.MotionMagic = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(Joint.CRUISE_VELOCITY)
            .withMotionMagicAcceleration(Joint.ACCELERATION)
            .withMotionMagicJerk(Joint.JERK);

        configuration.CurrentLimits.StatorCurrentLimit = Joint.STATOR_CURRENT_LIMT;
        configuration.CurrentLimits.SupplyCurrentLimit = Joint.SUPPLY_CURRENT_LIMIT;
        configuration.CurrentLimits.SupplyCurrentLowerTime = Joint.SUPPLY_CURRENT_LOWER_TIME;

        return configuration;
    }

    /**
     * Gets the {@link CANcoderConfiguration} for the joint CANCoder
     * 
     * @return the {@link CANcoderConfiguration} for the joint CANCoder
     */
    private CANcoderConfiguration jointCANCoderConfiguration() {
        CANcoderConfiguration configuration = new CANcoderConfiguration();

        configuration.MagnetSensor.MagnetOffset = Joint.MAGNET_OFFSET;

        return configuration;
    }

    /**
     * Gets the singleton instance of the {@link Arm} subsystem
     * 
     * @return the {@link Arm} subsystem
     */
    public static Arm system() {
        if (system == null) {
            system = new Arm();
        }

        return system;
    }
}