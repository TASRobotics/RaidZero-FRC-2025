package raidzero.robot.subsystems.telescopingarm;

import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.config.ServoChannelConfig.BehaviorWhenDisabled;
import com.revrobotics.servohub.config.ServoHubConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidzero.lib.LazyCan;
import raidzero.robot.Constants;
import raidzero.robot.Constants.TelescopingArm.Intake;

public class CoralIntake extends SubsystemBase {
    private TalonFXS roller, follow;

    private ServoHub servoHub;
    private ServoChannel intakeBlock;

    private LazyCan bottomLaser, topLaser;

    private static CoralIntake system;

    /**
     * Constructs a {@link CoralIntake} subsystem instance
     */
    private CoralIntake() {
        roller = new TalonFXS(Constants.TelescopingArm.Intake.MOTOR_ID);
        roller.getConfigurator().apply(rollerConfiguration());

        follow = new TalonFXS(Constants.TelescopingArm.Intake.FOLLOW_ID);
        follow.setControl(new Follower(Constants.TelescopingArm.Intake.MOTOR_ID, true));
        follow.getConfigurator().apply(followConfiguration());

        bottomLaser = new LazyCan(Constants.TelescopingArm.Intake.BOTTOM_LASERCAN_ID)
            .withRangingMode(RangingMode.SHORT)
            .withRegionOfInterest(8, 8, 4, 4)
            .withTimingBudget(TimingBudget.TIMING_BUDGET_20MS);

        topLaser = new LazyCan(Constants.TelescopingArm.Intake.TOP_LASERCAN_ID)
            .withRangingMode(RangingMode.SHORT)
            .withRegionOfInterest(8, 8, 4, 4)
            .withTimingBudget(TimingBudget.TIMING_BUDGET_20MS);

        servoHub = new ServoHub(Constants.TelescopingArm.Intake.SERVO_HUB_ID);
        servoHub.configure(getServoHubConfig(), ServoHub.ResetMode.kResetSafeParameters);

        intakeBlock = servoHub.getServoChannel(ChannelId.kChannelId2);
        intakeBlock.setPowered(true);
        intakeBlock.setEnabled(true);
    }

    /**
     * Gets the roller motor controller for disabled init to check for position
     *
     * @return The Roller motor
     */
    public TalonFXS getRoller() {
        return roller;
    }

    /**
     * Creates a {@link Command} to intake the coral
     *
     * @return A {@link Command} to intake the coral
     */
    public Command intake() {
        return run(() -> roller.set(Constants.TelescopingArm.Intake.INTAKE_SPEED))
            .until(() -> getBottomLaserDistance() <= Constants.TelescopingArm.Intake.LASERCAN_DISTANCE_THRESHOLD_MM);
    }

    /**
     * Creates a {@link Command} to intake the coral slower
     *
     * @return A {@link Command} to intake the coral slower
     */
    public Command contingencyIntake() {
        return startRun(
            () -> roller.getConfigurator().apply(
                rollerConfiguration().withCurrentLimits(
                    new CurrentLimitsConfigs().withSupplyCurrentLimit(3).withSupplyCurrentLowerLimit(1.5).withSupplyCurrentLowerTime(0.05)
                )
            ),
            () -> roller.set(Constants.TelescopingArm.Intake.INTAKE_SPEED - 0.05)
        ).until(() -> getBottomLaserDistance() <= Intake.LASERCAN_DISTANCE_THRESHOLD_MM)
            .finallyDo(() -> roller.getConfigurator().apply(rollerConfiguration()));
    }

    /**
     * Creates a {@link Command} to move the coral upwards to unstuck the servo block
     *
     * @return A {@link Command} to move the coral upwards
     */
    public Command unstuckServo() {
        return run(() -> roller.set(-Constants.TelescopingArm.Intake.INTAKE_LOWER_SPEED))
            .until(() -> getBottomLaserDistance() >= Constants.TelescopingArm.Intake.LASERCAN_DISTANCE_THRESHOLD_MM);
    }

    /**
     * Creates a {@link Command} to stop the intake
     *
     * @return A {@link Command} to stop the intake
     */
    public Command stop() {
        return runOnce(() -> roller.stopMotor());
    }

    /**
     * Creates a {@link Command} to extake the coral
     *
     * @return A {@link Command} to extake the coral
     */
    public Command extake() {
        return run(() -> roller.set(Constants.TelescopingArm.Intake.EXTAKE_SPEED))
            .withTimeout(Constants.TelescopingArm.Intake.EXTAKE_TIMEOUT_S);
    }

    /**
     * Creates a {@link Command} to run the roller at the specified speed
     *
     * @param speed The speed to run the roller at [-1, 1]
     * @return A {@link Command} to run the roller at the specified speed
     */
    public Command run(double speed) {
        return run(() -> roller.set(speed));
    }

    @Override
    public void periodic() {
        if (getBottomLaserDistance() > Constants.TelescopingArm.Intake.LASERCAN_DISTANCE_THRESHOLD_MM) {
            intakeBlock.setPulseWidth(Constants.TelescopingArm.Intake.SERVO_EXTENDED);
        } else {
            intakeBlock.setPulseWidth(Constants.TelescopingArm.Intake.SERVO_RETRACTED);
        }
    }

    /**
     * Gets the distance from the LaserCAN
     *
     * @return The distance in mm, -1 if the LaserCAN cannot be found
     */
    public int getTopLaserDistance() {
        return topLaser.getDistanceMm();
    }

    /**
     * Gets the distance from the LaserCAN
     *
     * @return The distance in mm, -1 if the LaserCAN cannot be found
     */
    public int getBottomLaserDistance() {
        return bottomLaser.getDistanceMm();
    }

    /**
     * Gets the {@link TalonFXSConfiguration} for the roller motor
     *
     * @return The {@link TalonFXSConfiguration} for the roller motor
     */
    private TalonFXSConfiguration rollerConfiguration() {
        TalonFXSConfiguration configuration = new TalonFXSConfiguration();

        configuration.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        return configuration;
    }

    /**
     * Gets the {@link TalonFXSConfiguration} for the roller follower
     *
     * @return The {@link TalonFXSConfiguration} for the roller follower
     */
    private TalonFXSConfiguration followConfiguration() {
        TalonFXSConfiguration configuration = new TalonFXSConfiguration();

        configuration.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        return configuration;
    }

    /**
     * Gets the {@link ServoHubConfig} for the REV Servo Hub
     *
     * @return The {@link ServoHubConfig} for the REV Servo Hub
     */
    private ServoHubConfig getServoHubConfig() {
        ServoHubConfig config = new ServoHubConfig();

        config.channel2
            .pulseRange(
                Constants.TelescopingArm.Intake.SERVO_EXTENDED,
                Constants.TelescopingArm.Intake.SERVO_CENTER_WIDTH,
                Constants.TelescopingArm.Intake.SERVO_RETRACTED
            )
            .disableBehavior(BehaviorWhenDisabled.kSupplyPower);

        return config;
    }

    /**
     * Gets the {@link CoralIntake} subsystem instance
     *
     * @return The {@link CoralIntake} subsystem instance
     */
    public static CoralIntake system() {
        if (system == null) {
            system = new CoralIntake();
        }

        return system;
    }
}
