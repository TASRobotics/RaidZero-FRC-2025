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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import raidzero.robot.subsystems.climb.ClimbJoint;
import raidzero.robot.lib.LazyTalon;
import raidzero.robot.Constants.TelescopingArm.Telescope;
import raidzero.robot.Constants.TelescopingArm.Joint;
import raidzero.robot.Constants.TelescopingArm.Positions;
import raidzero.robot.Constants.CANdle;

public class Arm extends SubsystemBase {
    private LazyTalon telescope, joint;

    private double[] currentPose;
    private double intakePosYOffset;

    private static Arm system;

    /**
     * Constructs an {@link Arm} subsystem instance
     */
    private Arm() {
        telescope = new LazyTalon(
            Telescope.MOTOR_ID, Telescope.CONVERSION_FACTOR,
            InvertedValue.CounterClockwise_Positive, Telescope.STATOR_CURRENT_LIMIT,
            Telescope.SUPPLY_CURRENT_LIMIT
        ).withMotionMagicConfiguration(
            Telescope.KP, Telescope.KI, Telescope.KD, Telescope.KS, Telescope.KG, Telescope.KV, Telescope.KA, Telescope.GRAVITY_TYPE_VALUE,
            Telescope.CRUISE_VELOCITY, Telescope.ACCELERATION
        ).withLimitSwitch(true, 0.0, false, 0.0).build();

        joint = new LazyTalon(
            Joint.MOTOR_ID, 1.0 / Joint.CANCODER_GEAR_RATIO, InvertedValue.Clockwise_Positive, Joint.STATOR_CURRENT_LIMT,
            Joint.SUPPLY_CURRENT_LIMIT
        ).withMotionMagicConfiguration(
            Joint.KP, Joint.KI, Joint.KD, Joint.KS, Joint.KG, Joint.KV, Joint.KA, Joint.GRAVITY_TYPE_VALUE, Joint.CRUISE_VELOCITY,
            Joint.ACCELERATION
        ).withCANCoder(
            Joint.CANCODER_ID, FeedbackSensorSourceValue.SyncCANcoder, Joint.CANCODER_OFFSET, SensorDirectionValue.Clockwise_Positive,
            Joint.CANCODER_DISCONTINUITY_POINT, Joint.CONVERSION_FACTOR * Joint.CANCODER_GEAR_RATIO
        ).build();

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
                    Commands.waitUntil(() -> joint.getFeedbackPosition() < 0.25)
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
     * @Note This method should only be used when lowering the arm
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
                Commands.waitUntil(() -> joint.getFeedbackPosition() < 0.25)
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
                    new double[] { Positions.INTAKE_POS_M_BLUE[0],
                        Positions.INTAKE_POS_M_BLUE[1] + intakePosYOffset }
                )
            );
        } else {
            return defer(
                () -> moveWithDelay(
                    new double[] { Positions.INTAKE_POS_M[0],
                        Positions.INTAKE_POS_M[1] + intakePosYOffset }
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
            return defer(() -> moveWithDelay(Positions.L4_SCORING_POS_M_BLUE));
        } else {
            return defer(() -> moveWithDelay(Positions.L4_SCORING_POS_M));

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
        telescope.moveTo(setpoint);
        SmartDashboard.putNumber("Telescope Setpoint", setpoint);
    }

    /**
     * Runs just the joint to the supplied setpoint
     * 
     * @param setpoint The target setpoint in rotations
     * @return A {@link Command} that moves the joint to the desired setpoint
     */
    public void moveJoint(double setpoint) {
        joint.moveTo(setpoint);
        SmartDashboard.putNumber("Joint Setpoint", setpoint);
    }

    /**
     * Updates the coast mode of the joint motor based on climb joint position
     * 
     * @Note This should only be called during disabled.
     */
    public void updateCoastMode() {
        if (shouldBeInCoast()) {
            joint.setNeutralMode(NeutralModeValue.Coast);
        } else {
            joint.setNeutralMode(NeutralModeValue.Brake);
        }
    }

    /**
     * Checks if the arm joint should be in coast mode
     * 
     * @return True if the arm joint should be in coast mode, false otherwise
     */
    private boolean shouldBeInCoast() {
        return (ClimbJoint.system().getPosition() < CANdle.CLIMB_JOINT_THRESHOLD);
    }

    /**
     * Checks if the arm is in a deployed height
     * 
     * @return True if the arm is in a deployed height, false otherwise
     */
    public boolean isUp() {
        return telescope.getFeedbackPosition() < -0.15;
    }

    /**
     * Calculates the target telescope position given the x and y setpoints
     * 
     * @param x The x setpoint in meters
     * @param y The y setpoint in meters
     * @return Target motor position in rotations
     */
    public double calculateTelescopeHeight(double[] position) {
        double height = Math.sqrt(Math.pow(position[0], 2) + Math.pow(position[1], 2)) - Telescope.GROUND_OFFSET_M;

        return height / Telescope.MAX_HEIGHT_M;
    }

    /**
     * Calculates the target arm position given the x and y setpoints
     * 
     * @param x The x setpoint in meters
     * @param y The y setpoint in meters
     * @return The target arm position in rotations
     */
    public double calculateJointAngle(double[] position) {
        return (Math.atan2(position[1], position[0])) * 180 / Math.PI / 360.0;
    }

    /**
     * Gets the telescope motor's encoder position
     * 
     * @return The telescope motor encoder position in rotations
     */
    public double getTelescopePosition() {
        return telescope.getFeedbackPosition();
    }

    /**
     * Gets the arm motor's encoder position
     * 
     * @return The arm motor encoder position in rotations
     */
    public double getJointPosition() {
        return joint.getFeedbackPosition();
    }

    /**
     * Stops the telescope motor
     */
    public void stopTelescope() {
        telescope.stop();
    }

    /**
     * Stops the arm motor
     */
    public void stopJoint() {
        joint.stop();
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