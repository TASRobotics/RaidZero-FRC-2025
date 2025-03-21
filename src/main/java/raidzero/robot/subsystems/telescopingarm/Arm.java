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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidzero.robot.Constants;
import raidzero.robot.subsystems.climb.ClimbJoint;

public class Arm extends SubsystemBase {
    private TalonFX telescope, joint;
    private CANcoder jointCANcoder;

    private double[] currentPose;
    private double intakePosYOffset;

    private static Arm system;

    /**
     * Constructs an {@link Arm} subsystem instance
     */
    private Arm() {
        telescope = new TalonFX(Constants.TelescopingArm.Telescope.MOTOR_ID);
        telescope.getConfigurator().apply(telescopeConfiguration());
        telescope.setNeutralMode(NeutralModeValue.Brake);

        joint = new TalonFX(Constants.TelescopingArm.Joint.MOTOR_ID);
        joint.getConfigurator().apply(jointConfiguration());
        joint.setNeutralMode(NeutralModeValue.Brake);

        jointCANcoder = new CANcoder(Constants.TelescopingArm.Joint.CANCODER_ID);
        jointCANcoder.getConfigurator().apply(jointCANCoderConfiguration());

        currentPose = new double[] { 0.0, 0.0 };
        intakePosYOffset = 0.0;
    }

    /**
     * Moves the arm to the desired x and y setpoints
     *
     * @param desiredPosition The desired x and y setpoints
     * @return A {@link Command} that moves the arm to the desired setpoints
     */
    public Command moveTo(double[] desiredPosition) {
        double telescopeSetpoint = -1 * calculateTelescopeHeight(desiredPosition);
        double jointSetpoint = calculateJointAngle(desiredPosition);

        currentPose = desiredPosition;

        if (currentPose[1] > desiredPosition[1]) {
            currentPose[1] = desiredPosition[1];

            return run(() -> moveJoint(jointSetpoint))
                .alongWith(
                    Commands.waitUntil(() -> joint.getPosition().getValueAsDouble() < 0.25)
                        .andThen(() -> moveTelescope(telescopeSetpoint))
                );
        } else {
            currentPose[1] = desiredPosition[1];

            return run(() -> moveTelescope(telescopeSetpoint))
                .alongWith(Commands.waitSeconds(0.1).andThen(() -> moveJoint(jointSetpoint)));
        }
    }

    /**
     * Moves the arm to the desired x and y setpoints without delay
     *
     * @param desiredPosition The desired x and y setpoints
     * @return A {@link Command} that moves the arm to the desired setpoints
     */
    public Command moveWithoutDelay(double[] desiredPosition) {
        double telescopeSetpoint = -1 * calculateTelescopeHeight(desiredPosition);
        double jointSetpoint = calculateJointAngle(desiredPosition);

        currentPose = desiredPosition;

        return run(() -> {
            moveTelescope(telescopeSetpoint);
            moveJoint(jointSetpoint);
        });
    }

    /**
     * Moves the arm to the desired x and y setpoints with a delay
     *
     * <p><strong>Note:</strong> This method should only be used when lowering the arm.</p>
     *
     * @param desiredPosition The desired x and y setpoints
     * @return A {@link Command} that moves the arm to the desired setpoints
     */
    public Command moveWithDelay(double[] desiredPosition) {
        double telescopeSetpoint = -1 * calculateTelescopeHeight(desiredPosition);
        double jointSetpoint = calculateJointAngle(desiredPosition);

        currentPose = desiredPosition;

        return run(() -> moveJoint(jointSetpoint))
            .alongWith(
                Commands.waitUntil(() -> joint.getPosition().getValueAsDouble() < 0.25)
                    .andThen(() -> moveTelescope(telescopeSetpoint))
            );
    }

    /**
     * Decreases the intake Y offset by a desired amount
     *
     * @param ammount The desired offset amount
     */
    public void decreaseIntakeYOffset(double ammount) {
        intakePosYOffset += ammount;
    }

    /**
     * Removes the intake y offset
     */
    public void removeIntakeOffset() {
        intakePosYOffset = 0;
    }

    /**
     * Moves the arm to the intake position
     *
     * @return A {@link Command} that moves the arm to the intake position
     */
    public Command moveToIntake() {
        if ((DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue)) {

            return defer(
                () -> moveWithDelay(
                    new double[] { Constants.TelescopingArm.Positions.INTAKE_POS_M_BLUE[0],
                        Constants.TelescopingArm.Positions.INTAKE_POS_M_BLUE[1] + intakePosYOffset }
                )
            );
        } else {
            return defer(
                () -> moveWithDelay(
                    new double[] { Constants.TelescopingArm.Positions.INTAKE_POS_M[0],
                        Constants.TelescopingArm.Positions.INTAKE_POS_M[1] + intakePosYOffset }
                )
            );

        }
    }

    /**
     * Moves the arm to the L4 scoring position
     *
     * @return A {@link Command} that moves the arm to the L4 scoring position
     */
    public Command moveToL4() {
        if ((DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue)) {
            return defer(() -> moveWithDelay(Constants.TelescopingArm.Positions.L4_SCORING_POS_M_BLUE));
        } else {
            return defer(() -> moveWithDelay(Constants.TelescopingArm.Positions.L4_SCORING_POS_M));

        }
    }

    /**
     * Moves the arm to a vertical position
     *
     * @return A {@link Command} that moves the arm to a vertical position
     */
    public Command climbPos() {
        return run(() -> {
            moveJoint(0.3);
            moveTelescope(0.0);
        });
    }

    /**
     * Runs just the telescope to the supplied setpoint
     *
     * @param setpoint The target setpoint in percentage of full range of motion
     */
    public void moveTelescope(double setpoint) {
        telescope.setControl(new MotionMagicVoltage(0).withPosition(setpoint));
        SmartDashboard.putNumber("Telescope Setpoint", setpoint);
    }

    /**
     * Runs just the joint to the supplied setpoint
     *
     * @param setpoint The target setpoint in rotations
     */
    public void moveJoint(double setpoint) {
        joint.setControl(new MotionMagicVoltage(0).withPosition(setpoint));
        SmartDashboard.putNumber("Joint Setpoint", setpoint);
    }

    /**
     * Updates the coast mode of the joint motor based on climb joint position
     *
     * <p><strong>Note:</strong> This should only be called during disabled.</p>
     */
    public void updateCoastMode() {
        if (shouldBeInCoast()) {
            joint.setNeutralMode(NeutralModeValue.Coast);
            // telescope.setNeutralMode(NeutralModeValue.Coast);
        } else {
            joint.setNeutralMode(NeutralModeValue.Brake);
            // telescope.setNeutralMode(NeutralModeValue.Brake);
        }
    }

    /**
     * Checks if the arm joint should be in coast mode
     *
     * @return True if the arm joint should be in coast mode, false otherwise
     */
    private boolean shouldBeInCoast() {
        return (ClimbJoint.system().getPosition() < Constants.CANdle.CLIMB_JOINT_THRESHOLD);
    }

    /**
     * Zeroes the the relative encoder position in the telescope motor
     *
     * @return A {@link Command} that zeroes the telescope motor position
     */
    public Command zeroTelescopePosition() {
        return new InstantCommand(() -> telescope.setPosition(0));
    }

    /**
     * Checks if the arm is in a deployed height
     *
     * @return True if the arm is in a deployed height, false otherwise
     */
    public boolean isUp() {
        return telescope.getPosition().getValueAsDouble() < -0.15;
    }

    /**
     * Calculates the target telescope position given the x and y setpoints
     *
     * @param position The desired position in meters
     * @return Target motor position in rotations
     */
    public double calculateTelescopeHeight(double[] position) {
        double height = Math.sqrt(Math.pow(position[0], 2) + Math.pow(position[1], 2)) - Constants.TelescopingArm.Telescope.GROUND_OFFSET_M;

        return height / Constants.TelescopingArm.Telescope.MAX_HEIGHT_M;
    }

    /**
     * Calculates the target arm position given the x and y setpoints
     *
     * @param position The desired position in meters
     * @return The target arm position in rotations
     */
    public double calculateJointAngle(double[] position) {
        return Math.atan2(position[1], position[0]) * 180 / Math.PI / 360.0;
    }

    /**
     * Gets the telescope motor's encoder position
     *
     * @return The telescope motor encoder position in rotations
     */
    public double getTelescopePosition() {
        return telescope.getPosition().getValueAsDouble();
    }

    /**
     * Gets the arm motor's encoder position
     *
     * @return The arm motor encoder position in rotations
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
    public void stopJoint() {
        joint.stopMotor();
    }

    /**
     * Stops all motors
     */
    public void stopAll() {
        stopTelescope();
        stopJoint();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator pos", getTelescopePosition());
        SmartDashboard.putNumber("Intake Y Offset", Math.round(intakePosYOffset * 100) / 100.0);
    }

    /**
     * Gets the {@link TalonFXConfiguration} for the telescope
     *
     * @return The {@link TalonFXConfiguration} for the telescope
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

        configuration.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        configuration.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0.0;
        configuration.HardwareLimitSwitch.ReverseLimitEnable = false;

        configuration.Feedback.SensorToMechanismRatio = Constants.TelescopingArm.Telescope.CONVERSION_FACTOR;

        configuration.Slot0.GravityType = Constants.TelescopingArm.Telescope.GRAVITY_TYPE_VALUE;

        return configuration;
    }

    /**
     * Gets the {@link TalonFXConfiguration} for the arm joint
     *
     * @return The {@link TalonFXConfiguration} for the arm joint
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
     * @return The {@link CANcoderConfiguration} for the joint CANCoder
     */
    private CANcoderConfiguration jointCANCoderConfiguration() {
        CANcoderConfiguration configuration = new CANcoderConfiguration();

        configuration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = Constants.TelescopingArm.Joint.CANCODER_DISCONTINUITY_POINT;
        configuration.MagnetSensor.MagnetOffset = Constants.TelescopingArm.Joint.CANCODER_OFFSET;
        configuration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        return configuration;
    }

    /**
     * Gets the {@link Arm} subsystem instance
     *
     * @return The {@link Arm} subsystem instance
     */
    public static Arm system() {
        if (system == null) {
            system = new Arm();
        }

        return system;
    }
}