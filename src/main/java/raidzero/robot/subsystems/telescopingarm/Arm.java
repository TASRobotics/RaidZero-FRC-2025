package raidzero.robot.subsystems.telescopingarm;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
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
import raidzero.robot.Constants.TelescopingArm.Telescope;
import raidzero.robot.subsystems.climb.ClimbJoint;

public class Arm extends SubsystemBase {
    private TalonFX telescope, joint;
    private CANcoder jointCANcoder;

    private double[] currentPose;
    private double intakePosYOffset;

    private boolean jointNeutralMode;

    private static Arm system;

    /**
     * Constructs an {@link Arm} subsystem instance
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

        currentPose = new double[] { 0.0, 0.0 };
        intakePosYOffset = 0.0;
        jointNeutralMode = true;
    }

    public Command moveTheArmInAStraightLineUsingDifferentialTransformations(double[] desiredPosition, double[] cartesianVelocities) {
        double r = (this.getTelescopePosition() * Telescope.CONVERSION_FACTOR);
        double theta = this.getJointPosition() * (2.0 * Math.PI);

        double[][] rotationMatix = new double[][] {
            { Math.cos(theta), Math.sin(theta) },
            { -1.0 * Math.sin(theta), Math.cos(theta) },
        };

        double[] polarVelocities = matrixMultiplication(rotationMatix, cartesianVelocities);

        double telescopeVelocity = polarVelocities[0] / (Telescope.MAX_HEIGHT_M - Telescope.MIN_HEIGHT_M);
        double jointVelocity = (polarVelocities[1] / r) / (2.0 * Math.PI);

        return defer(
            () -> this.moveWithVelocities(jointVelocity, telescopeVelocity).until(armWithinSetpoint(desiredPosition))
                .andThen(moveWithoutDelay(desiredPosition))
        );

        // var telescopeConfig = telescopeConfiguration();
        // telescopeConfig.MotionMagic.MotionMagicCruiseVelocity = telescopeVelocity;
        // telescope.getConfigurator().apply(telescopeConfig);

        // var jointConfig = jointConfiguration();
        // jointConfig.MotionMagic.MotionMagicCruiseVelocity = jointVelocity;
        // joint.getConfigurator().apply(jointConfig);

        // return defer(
        // () -> moveWithoutDelay(desiredPosition)
        // );
    }

    private BooleanSupplier armWithinSetpoint(double[] setpoint) {
        double[] position = calculateCurrentPosition();

        return () -> Math.abs(position[0] - setpoint[0]) < 0.02 || Math.abs(position[1] - setpoint[1]) < 0.02;
    }

    public Command moveWithVelocities(double jointVelocity, double telescopeVelocity) {
        return run(() -> {
            joint.setControl(new MotionMagicVelocityVoltage(jointVelocity));
            telescope.setControl(new MotionMagicVelocityVoltage(telescopeVelocity));
        });
    }

    public double[] matrixMultiplication(double[][] a, double[] b) {
        double[] result = new double[2];
        result[0] = a[0][0] * b[0] + a[0][1] * b[1];
        result[1] = a[1][0] * b[0] + a[1][1] * b[1];
        return result;
    }

    /**
     * Moves the arm to the desired x and y setpoints
     * 
     * @param desiredPosition The desired x and y setpoints
     * @return A {@link Command} that moves the arm to the desired setpoints
     */
    public Command moveTo(double[] desiredPosition) {
        double telescopeSetpoint = calculateTelescopeHeight(desiredPosition);
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
        double telescopeSetpoint = calculateTelescopeHeight(desiredPosition);
        double jointSetpoint = calculateJointAngle(desiredPosition);

        currentPose = desiredPosition;

        return run(() -> {
            moveTelescope(telescopeSetpoint);
            moveJoint(jointSetpoint);
        });
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
                () -> moveTo(
                    new double[] { Constants.TelescopingArm.Positions.INTAKE_POS_M_BLUE[0],
                        Constants.TelescopingArm.Positions.INTAKE_POS_M_BLUE[1] + intakePosYOffset }
                )
            );
        } else {
            return defer(
                () -> moveTo(
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
            return defer(() -> moveTo(Constants.TelescopingArm.Positions.L4_SCORING_POS_M_BLUE));
        } else {
            return defer(() -> moveTo(Constants.TelescopingArm.Positions.L4_SCORING_POS_M));
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
     * @return A {@link Command} that moves the telescope to the desired setpoint
     */
    public void moveTelescope(double setpoint) {
        telescope.setControl((new MotionMagicVoltage(0)).withPosition(setpoint));
        SmartDashboard.putNumber("Telescope Setpoint", setpoint);
    }

    /**
     * Runs just the joint to the supplied setpoint
     * 
     * @param setpoint The target setpoint in rotations
     * @return A {@link Command} that moves the joint to the desired setpoint
     */
    public void moveJoint(double setpoint) {
        joint.setControl((new MotionMagicVoltage(0)).withPosition(setpoint));
        SmartDashboard.putNumber("Joint Setpoint", setpoint);
    }

    /**
     * Updates the coast mode of the joint motor based on climb joint position
     * 
     * @Note This should only be called during disabled.
     */
    public void updateCoastMode() {
        if (shouldBeInCoast() && jointNeutralMode == true) {
            joint.setNeutralMode(NeutralModeValue.Coast);
            jointNeutralMode = false;
        } else if (jointNeutralMode == false) {
            joint.setNeutralMode(NeutralModeValue.Brake);
            jointNeutralMode = true;
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
        return telescope.getPosition().getValueAsDouble() < 0.15;
    }

    /**
     * Calculates the target telescope position given the x and y setpoints
     * 
     * @param x The x setpoint in meters
     * @param y The y setpoint in meters
     * @return Target motor position in rotations
     */
    public double calculateTelescopeHeight(double[] position) {
        double height = Math.sqrt(Math.pow(position[0], 2) + Math.pow(position[1], 2)) - Constants.TelescopingArm.Telescope.MIN_HEIGHT_M;

        return height / Telescope.MAX_MINUS_MIN_M;
    }

    /**
     * Calculates the target arm position given the x and y setpoints
     * 
     * @param x The x setpoint in meters
     * @param y The y setpoint in meters
     * @return The target arm position in rotations
     */
    public double calculateJointAngle(double[] position) {
        return (Math.atan2(position[1], position[0])) / (2.0 * Math.PI);
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
     * @return The arm motor encoder position in relative position as a percentage of full extension
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

    /**
     * Calculates the current x and y septoint in meters from the joint angle and telescope extension.
     * 
     * @return the calculated setpoint in meters
     */
    public double[] calculateCurrentPosition() {
        double r = (this.getTelescopePosition() * Telescope.MAX_MINUS_MIN_M + Telescope.MIN_HEIGHT_M);
        double theta = this.getJointPosition() * (2.0 * Math.PI);

        return new double[] { r * Math.cos(theta), r * Math.sin(theta) };
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Y Offset", Math.round(intakePosYOffset * 100) / 100.0);

        SmartDashboard.putNumber("Calculated X", calculateCurrentPosition()[0]);
        SmartDashboard.putNumber("Calculated Y", calculateCurrentPosition()[1]);
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

        configuration.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;
        configuration.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = 0.0;
        configuration.HardwareLimitSwitch.ForwardLimitEnable = false;

        configuration.Feedback.SensorToMechanismRatio = Constants.TelescopingArm.Telescope.CONVERSION_FACTOR;

        configuration.Slot0.GravityType = Constants.TelescopingArm.Telescope.GRAVITY_TYPE_VALUE;

        configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

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