package raidzero.robot.subsystems.telescopingarm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import raidzero.robot.Constants;

public class Arm extends SubsystemBase {
    private static Arm system;

    private TalonFX telescope, joint;
    private CANcoder jointCANcoder;

    /**
     * Constructor for the {@link Arm} subsystem
     */
    private Arm() {
        telescope = new TalonFX(Constants.TelescopingArm.Telescope.MOTOR_ID);
        telescope.getConfigurator().apply(telescopeConfiguration());
        telescope.setNeutralMode(NeutralModeValue.Brake);

        joint = new TalonFX(Constants.TelescopingArm.Joint.MOTOR_ID);
        joint.getConfigurator().apply((jointConfiguration()));
        joint.setNeutralMode(NeutralModeValue.Brake);

        jointCANcoder = new CANcoder(Constants.TelescopingArm.Joint.CANCODER_ID);
        jointCANcoder.getConfigurator().apply(jointCANCoderConfiguration());

    }

    /**
     * Creates a {@link Command} to move the arm to the specified x and y setpoints
     * 
     * @param x the x setpoint in meters
     * @param y the y setpoint in meters
     * @return the command to be scheudled and run
     */
    public Command moveArm(double x, double y) {
        double telescopeSetpoint = -1 * calculateTelescopeHeight(x, y);
        double jointSetpoint = calculateJointAngle(x, y);

        return run(() -> moveTelescope(telescopeSetpoint))
            .alongWith(Commands.waitSeconds(0.1).andThen(() -> moveJoint(jointSetpoint)));
    }

    public Command goToIntakePos() {
        double telescopeSetpoint = -1 * calculateTelescopeHeight(
            Constants.TelescopingArm.Positions.INTAKE_POS_M[0], Constants.TelescopingArm.Positions.INTAKE_POS_M[1]
        );
        double jointSetpoint = calculateJointAngle(
            Constants.TelescopingArm.Positions.INTAKE_POS_M[0], Constants.TelescopingArm.Positions.INTAKE_POS_M[1]
        );

        return run(() -> {
            moveTelescope(telescopeSetpoint);
            moveJoint(jointSetpoint);
        });
    }

    public Command moveArmWithRotations(double jointSetpoint, double telescopeSetpoint) {
        return run(() -> moveJoint(jointSetpoint))
            .alongWith(
                Commands.waitUntil(() -> joint.getPosition().getValueAsDouble() < 0.25)
                .andThen(() -> moveTelescope(telescopeSetpoint)));
    }

    /**
     * Runs just the telescope to the supplied setpoint
     * 
     * @param setpoint the target setpoint in percentage of full range of motion
     * @return the command to be scheduled and run
     */
    public Command runTelescope(double setpoint) {
        return run(() -> moveTelescope(setpoint));
    }

    public Command runJoint(double setpoint) {
        return run(() -> moveJoint(setpoint));
    }

    /**
     * Zeroes the the relative encoder position in the telescope motor
     * 
     * @return the command to zero the position
     */
    public Command zeroTelescopePosition() {
        return new InstantCommand(() -> telescope.setPosition(0));
    }

    private void moveTelescope(double setpoint) {
        final MotionMagicVoltage request = new MotionMagicVoltage(0);
        telescope.setControl(request.withPosition(setpoint));
        SmartDashboard.putNumber("Telescope Setpoint", setpoint);
    }

    private void moveJoint(double setpoint) {
        final MotionMagicVoltage request = new MotionMagicVoltage(0);
        joint.setControl(request.withPosition(setpoint));
        SmartDashboard.putNumber("Joint Setpoint", setpoint);
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
    private double calculateTelescopeHeight(double x, double y) {
        double height = Math.sqrt(x * x + y * y);
        height -= Constants.TelescopingArm.Telescope.GROUND_OFFSET_M;

        return height / Constants.TelescopingArm.Telescope.MAX_HEIGHT_M;
    }

    /**
     * Calculates the target arm position given the x and y setpoints
     * 
     * @param x the x setpoint in meters
     * @param y the y setpoint in meters
     * @return the target arm position in rotations
     */
    public double calculateJointAngle(double x, double y) {
        return (Math.atan2(y, x)) * 180 / Math.PI / 360.0;
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

    public Command stopAllCommand() {
        return run(() -> stopAll());
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
    private TalonFXConfiguration jointConfiguration() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        configuration.Feedback.SensorToMechanismRatio = 1.0 / Constants.TelescopingArm.Joint.CANCODER_GEAR_RATIO;
        configuration.Feedback.RotorToSensorRatio = Constants.TelescopingArm.Joint.CONVERSION_FACTOR *
            Constants.TelescopingArm.Joint.CANCODER_GEAR_RATIO;

        configuration.Slot0 = new Slot0Configs()
            .withKS(Constants.TelescopingArm.Joint.KS)
            .withKV(Constants.TelescopingArm.Joint.KV)
            .withKA(Constants.TelescopingArm.Joint.KA)
            .withKG(Constants.TelescopingArm.Joint.KG)
            .withKP(Constants.TelescopingArm.Joint.KP)
            .withKI(Constants.TelescopingArm.Joint.KI)
            .withKD(Constants.TelescopingArm.Joint.KD);

        configuration.Slot0.GravityType = Constants.TelescopingArm.Joint.GRAVITY_TYPE_VALUE;

        configuration.MotionMagic = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(Constants.TelescopingArm.Joint.CRUISE_VELOCITY)
            .withMotionMagicAcceleration(Constants.TelescopingArm.Joint.ACCELERATION)
            .withMotionMagicJerk(Constants.TelescopingArm.Joint.JERK);

        configuration.CurrentLimits.StatorCurrentLimit = Constants.TelescopingArm.Joint.STATOR_CURRENT_LIMT;
        configuration.CurrentLimits.SupplyCurrentLimit = Constants.TelescopingArm.Joint.SUPPLY_CURRENT_LIMIT;
        configuration.CurrentLimits.SupplyCurrentLowerTime = Constants.TelescopingArm.Joint.SUPPLY_CURRENT_LOWER_TIME;

        configuration.Feedback.FeedbackRemoteSensorID = Constants.TelescopingArm.Joint.CANCODER_ID;
        configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;

        configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        return configuration;
    }

    /**
     * Gets the {@link CANcoderConfiguration} for the joint CANCoder
     * 
     * @return the {@link CANcoderConfiguration} for the joint CANCoder
     */
    private CANcoderConfiguration jointCANCoderConfiguration() {
        CANcoderConfiguration configuration = new CANcoderConfiguration();

        configuration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = Constants.TelescopingArm.Joint.CANCODER_DISCONTINUITY_POINT;
        configuration.MagnetSensor.MagnetOffset = Constants.TelescopingArm.Joint.CANCODER_OFFSET;
        configuration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        return configuration;
    }

    /**
     * Gets the singleton instance of the {@link Arm} subsystem
     * 
     * @return the {@link Arm} subsystem
     */
    public static Arm system() {
        if (system == null)
            system = new Arm();
        return system;
    }
}