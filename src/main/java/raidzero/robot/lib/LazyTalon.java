package raidzero.robot.lib;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class LazyTalon {
    private TalonFX motor, follower;
    private int motorID;
    private TalonFXConfiguration motorConfiguration, followConfiguration;

    private CANcoder canCoder;
    private CANcoderConfiguration canCoderConfiguration;

    /**
     * Constructs a LazyTalon instance with the specified motor identifier and configuration parameters.
     * 
     * <p>This constructor initializes the TalonFX motor controller with custom settings including sensor-to-mechanism
     * ratio, motor inversion, and current limits for both the stator and supply. These settings ensure the motor
     * behaves as expected for its role in robotic applications.
     * 
     * @param motorID                 the unique identifier for the motor controller
     * @param sensorToMechanismRatio  the ratio used to convert sensor readings into mechanism movement units
     * @param invertedValue           the inversion configuration for the motor output, which determines the direction of rotation
     * @param statorCurrentLimit      the maximum allowable current for the stator
     * @param supplyCurrentLimit      the maximum allowable current from the power supply
     */
    public LazyTalon(int motorID, double sensorToMechanismRatio, InvertedValue invertedValue, double statorCurrentLimit, double supplyCurrentLimit) {
        this.motorID = motorID;

        motorConfiguration = new TalonFXConfiguration();
        motorConfiguration.Feedback.SensorToMechanismRatio = sensorToMechanismRatio;

        motorConfiguration.MotorOutput.Inverted = invertedValue;

        motorConfiguration.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
        motorConfiguration.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
        motorConfiguration.CurrentLimits.SupplyCurrentLowerTime = 0.0;

        motor = new TalonFX(motorID);
        motor.setNeutralMode(NeutralModeValue.Brake);
    }

    /**
     * Configures a follower TalonFX for this LazyTalon.
     *
     * <p>This method creates a new TalonFX follower with the specified follower device ID,
     * applies a default configuration to it, and sets the follower control mode relative
     * to the master motor (represented by motorID). The follower's inversion state is also
     * configured based on the provided flag.
     *
     * @param followerID the device ID of the follower TalonFX
     * @param isInverted if true, the follower output is inverted relative to the master
     * @return this LazyTalon instance for method chaining
     */
    public LazyTalon withFollower(int followerID, boolean isInverted) {
        followConfiguration = new TalonFXConfiguration();

        follower = new TalonFX(followerID);
        follower.getConfigurator().apply(followConfiguration);
        follower.setControl(new Follower(motorID, isInverted));

        return this;
    }

    /**
     * Configures the LazyTalon with a CANCoder feedback sensor.
     * 
     * <p>This method initializes and applies the CANCoder configuration including magnet offset and sensor direction,
     * sets up the remote sensor feedback for the motor controller, and enables method chaining by returning this instance.</p>
     *
     * @param CANCoderID      the unique identifier of the CANCoder device.
     * @param sensorType      the type of sensor used for feedback.
     * @param magnetOffset    the magnet offset value applied to the sensor.
     * @param sensorDirection the sensor direction relative to the motor.
     * @return                this LazyTalon instance with the updated configuration.
     */
    public LazyTalon withCANCoder(int CANCoderID, FeedbackSensorSourceValue sensorType, double magnetOffset, SensorDirectionValue sensorDirection) {
        canCoderConfiguration = new CANcoderConfiguration();
        canCoderConfiguration.MagnetSensor.MagnetOffset = magnetOffset;
        canCoderConfiguration.MagnetSensor.SensorDirection = sensorDirection;

        canCoder = new CANcoder(CANCoderID);
        canCoder.getConfigurator().apply(canCoderConfiguration);

        motorConfiguration.Feedback.FeedbackRemoteSensorID = CANCoderID;
        motorConfiguration.Feedback.FeedbackSensorSource = sensorType;

        return this;
    }

    /**
     * Configures the CANCoder sensor with extended parameters and applies the configuration.
     *
     * <p>
     * This method delegates the basic CANCoder configuration to an overloaded method that handles the CANCoder ID,
     * sensor type, magnet offset, and sensor direction. After the basic configuration, it sets the absolute sensor
     * discontinuity point to handle wrap-around behavior before applying the complete configuration using the
     * device's configurator.
     * </p>
     *
     * @param CANCoderID the unique identifier of the CANCoder on the CAN bus.
     * @param sensorType the type of feedback sensor used, specified by a FeedbackSensorSourceValue.
     * @param magnetOffset the offset to be applied to the magnet sensor's position.
     * @param sensorDirection the direction setting for the sensor, specified by a SensorDirectionValue.
     * @param discontinuityPoint the point at which the sensor's absolute measurement wraps around.
     * @return the current instance of LazyTalon to allow for method chaining.
     */
    public LazyTalon withCANCoder(int CANCoderID, FeedbackSensorSourceValue sensorType, double magnetOffset, SensorDirectionValue sensorDirection, double discontinuityPoint, double rotorToSensorRatio) {
        withCANCoder(CANCoderID, sensorType, magnetOffset, sensorDirection);

        canCoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = discontinuityPoint;
        canCoder.getConfigurator().apply(canCoderConfiguration);

        motorConfiguration.Feedback.RotorToSensorRatio = rotorToSensorRatio;

        return this;
    }

    /**
     * Configures the motion magic parameters for this LazyTalon instance.
     *
     * <p>This method sets up the PIDF gains and motion magic parameters for controlling
     * the motor's movement using motion profiling. Specifically, it assigns the proportional,
     * integral, derivative, static, gravity, velocity, and acceleration gains for the PID controller,
     * as well as the motion magic cruise velocity and maximum acceleration values. The gravity type
     * is also specified.
     *
     * @param p the proportional gain (kP)
     * @param i the integral gain (kI)
     * @param d the derivative gain (kD)
     * @param s the static gain (kS)
     * @param g the gravity gain (kG)
     * @param v the velocity gain (kV)
     * @param a the acceleration gain (kA)
     * @param gravityType the gravity type configuration for the motor
     * @param cruiseVelocity the cruise velocity for motion magic mode
     * @param maxAcceleration the maximum acceleration for motion magic mode
     * @return the current instance of LazyTalon with updated motion magic configuration, enabling method chaining
     */
    public LazyTalon withMotionMagicConfiguration(double p, double i, double d, double s, double g, double v, double a, GravityTypeValue gravityType, double cruiseVelocity, double maxAcceleration) {
        motorConfiguration.Slot0.kP = p;
        motorConfiguration.Slot0.kI = i;
        motorConfiguration.Slot0.kD = d;
        motorConfiguration.Slot0.kS = s;
        motorConfiguration.Slot0.kG = g;
        motorConfiguration.Slot0.kV = v;
        motorConfiguration.Slot0.kA = a;
        motorConfiguration.Slot0.GravityType = gravityType;

        motorConfiguration.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
        motorConfiguration.MotionMagic.MotionMagicAcceleration = maxAcceleration;

        return this;
    }

    /**
     * Applies a custom configuration to this TalonFX motor controller.
     * <p>
     * This method sets the motor configuration to the provided {@link TalonFXConfiguration}
     * instance and returns the current instance of LazyTalon to enable method chaining.
     *
     * @param configuration the TalonFXConfiguration to be applied to the motor
     * @return the current instance of LazyTalon with the updated configuration
     */
    public LazyTalon withCustomConfiguration(TalonFXConfiguration configuration) {
        motorConfiguration = configuration;

        return this;
    }

    /**
     * Configures the hardware limit switch settings for the motor.
     *
     * <p>This method updates the forward and reverse limit switch configuration using the provided parameters.
     *
     * @param ForwardLimitAutosetPositionEnable  if true, enables automatic setting of the forward limit switch position.
     * @param ForwardLimitAutosetPositionValue   the position value to be used for the forward limit switch.
     * @param reverseLimitAutosetPositionEnable  if true, enables automatic setting of the reverse limit switch position.
     * @param reverseLimitAutosetPositionValue   the position value to be used for the reverse limit switch.
     * @return this LazyTalon instance with the updated limit switch configuration.
     */
    public LazyTalon withLimitSwitch(boolean forwardLimitAutosetPositionEnable, double forwardLimitAutosetPositionValue, boolean reverseLimitAutosetPositionEnable, double reverseLimitAutosetPositionValue) {
        motorConfiguration.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = forwardLimitAutosetPositionEnable;
        motorConfiguration.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = forwardLimitAutosetPositionValue;

        motorConfiguration.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = reverseLimitAutosetPositionEnable;
        motorConfiguration.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = reverseLimitAutosetPositionValue;

        return this;
    }

    /**
     * Configures the forward and reverse soft limits for the motor.
     *
     * <p>This method enables or disables the software limit switches for both the forward and
     * reverse directions and sets their respective threshold values.
     *
     * @param forwardSoftLimitEnable true to enable the forward soft limit; false to disable it.
     * @param forwardSoftLimit the threshold value for the forward soft limit.
     * @param reverseSoftLimitEnable true to enable the reverse soft limit; false to disable it.
     * @param reverseSoftLimit the threshold value for the reverse soft limit.
     * @return the current instance of LazyTalon with updated soft limit settings.
     */
    public LazyTalon withSoftLimits(boolean forwardSoftLimitEnable, double forwardSoftLimit, boolean reverseSoftLimitEnable, double reverseSoftLimit) {
        motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = forwardSoftLimitEnable;
        motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardSoftLimit;

        motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverseSoftLimitEnable;
        motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseSoftLimit;

        return this;
    }

    /**
     * Applies the motor configuration to the motor configurator and returns the current
     * instance of LazyTalon.
     *
     * @return this LazyTalon instance for method chaining.
     */
    public LazyTalon build() {
        motor.getConfigurator().apply(motorConfiguration);

        return this;
    }

    /**
     * Moves the motor to the specified position using a motion magic voltage profile.
     *
     * @param setpoint the target position setpoint for the motion magic control.
     */
    public void moveTo(double setpoint) {
        motor.setControl(new MotionMagicVoltage(setpoint));
    }

    /**
     * Moves the motor at the specified velocity using a motion magic voltage profile.
     * 
     * @param velocity the target velocity for the motion magic control. 
     */
    public void moveWithVelocity(double velocity) {
        motor.setControl(new MotionMagicVelocityVoltage(velocity));
    }

    /**
     * Stops the motor
     */
    public void stop() {
        motor.stopMotor();
    }

    /**
     * Sets the neutral mode for the motor.
     *
     * @param newNeutralModeValue the neutral mode to be applied to the motor
     */
    public void setNeutralMode(NeutralModeValue newNeutralModeValue) {
        motor.setNeutralMode(newNeutralModeValue);
    }

    /**
     * Retrieves the current feedback position of the motor.
     *
     * @return the current motor position as a double. 
     */
    public double getFeedbackPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    /**
     * Retrieves the current velocity feedback from the motor.
     *
     * @return the current feedback velocity from the motor as a double.
     */
    public double getFeedbackVelocity() {
        return motor.getVelocity().getValueAsDouble();
    }

    /**
     * Retrives the current acceleration feedback from the motor.
     * 
     * @return the current feedback velocity from the motor as a double.
     */
    public double getFeedbackAcceleration() {
        return motor.getAcceleration().getValueAsDouble();
    }

    /**
     * Retrieves the TalonFX motor instance.
     *
     * @return the TalonFX object associated with this instance.
     */
    public TalonFX getMotor() {
        return motor;
    }

    /**
     * Retrieves the CANcoder instance.
     *
     * @return the CANcoder instance associated with this object
     */
    public CANcoder getCanCoder() {
        return canCoder;
    }

    /**
     * Retrieves the follower TalonFX instance.
     *
     * @return the follower TalonFX used for motor control.
     */
    public TalonFX getFollower() {
        return follower;
    }
}