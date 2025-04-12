package raidzero.robot.subsystems.telescopingarm;

import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidzero.lib.LazyCan;
import raidzero.robot.Constants;
import raidzero.robot.Constants.TelescopingArm.Intake;

public class CoralIntake extends SubsystemBase {
    private TalonFXS roller, follower;
    private LazyCan bottomLaser, topLaser;

    private static CoralIntake system;

    /**
     * Constructs a {@link CoralIntake} subsystem instance
     */
    private CoralIntake() {
        roller = new TalonFXS(Constants.TelescopingArm.Intake.MOTOR_ID, Constants.RIO_BUS);
        roller.getConfigurator().apply(rollerConfiguration());

        follower = new TalonFXS(13);
        follower.getConfigurator().apply(followerConfiguration());
        follower.setControl(new Follower(Intake.MOTOR_ID, false));

        bottomLaser = new LazyCan(1).withRangingMode(RangingMode.SHORT)
            .withRegionOfInterest(14, 8, 16, 16).withTimingBudget(TimingBudget.TIMING_BUDGET_20MS)
            .withThreshold(Intake.BOTTOM_LASER_THRESHOLD_MM);

        topLaser = new LazyCan(0).withRangingMode(RangingMode.LONG)
            .withRegionOfInterest(8, 14, 16, 4).withTimingBudget(TimingBudget.TIMING_BUDGET_20MS)
            .withThreshold(Intake.TOP_LASER_THRESHOLD_MM);
    }

    /**
     * Intakes a coral
     *
     * @return A {@link Command} that intakes a coral
     */
    public Command intake() {
        return run(() -> roller.set(Intake.INTAKE_SPEED)).until(() -> bottomLaser.withinThreshold());
    }

    /**
     * Intakes a coral until the top laser is triggered
     *
     * <p><strong>Note:</strong> This method should only be used during the autonomous period.</p>
     *
     * @return A {@link Command} that intakes a coral until the top laser is triggered
     */
    public Command autoIntakeP1() {
        return run(() -> roller.set(Intake.INTAKE_SPEED)).until(() -> topLaser.withinThreshold());
    }

    /**
     * Intakes a coral until the bottom laser is triggered
     *
     * <p><strong>Note:</strong> This method should only be used during the autonomous period.</p>
     *
     * @return A {@link Command} that intakes a coral until the bottom laser is triggered
     */
    public Command autoIntakeP2() {
        return run(() -> roller.set(Intake.INTAKE_SPEED)).until(() -> bottomLaser.withinThreshold());
    }

    /**
     * Intakes an lgae
    
     * @return A {@link Command} that intakes an algae
     */
    public Command intakeAlgae() {
        return run(() -> roller.set(Intake.INTAKE_SPEED));
    }

    /**
     * Extakes an algae
     *
     * @return A {@link Command} that extakes an algae
     */
    public Command extakeAlgae() {
        return run(() -> roller.set(Intake.ALGAE_EJECT_SPEED));
    }

    /**
     * Holds the algae by applying a small amount of voltage
     *
     * @return A {@link Command} that holds the algae
     */
    public Command holdAlgae() {
        return run(() -> roller.set(Intake.HOLD_SPEED));
    }

    /**
     * Stops the intake
     *
     * @return A {@link Command} that stops the intake
     */
    public Command stop() {
        return runOnce(() -> roller.stopMotor());
    }

    /**
     * Extakes a coral
     *
     * @return A {@link Command} that extakes a coral
     */
    public Command extake() {
        return run(() -> roller.set(Constants.TelescopingArm.Intake.EXTAKE_SPEED))
            .withTimeout(Constants.TelescopingArm.Intake.EXTAKE_TIMEOUT_S);
    }

    /**
     * Runs the roller at the specified speed
     *
     * @param speed The speed to run the roller at [-1, 1]
     * @return A {@link Command} to run the roller at the specified speed
     */
    public Command run(double speed) {
        return run(() -> roller.set(speed));
    }

    /**
     * Updates the coast mode of the roller and follower motors
     *
     * <p><strong>Note:</strong> This should only be called during disabled.</p>
     */
    public void updateCoastMode() {
        if (shouldBeInCoast()) {
            roller.setNeutralMode(NeutralModeValue.Coast);
            follower.setNeutralMode(NeutralModeValue.Coast);
        } else {
            roller.setNeutralMode(NeutralModeValue.Brake);
            follower.setNeutralMode(NeutralModeValue.Brake);
        }
    }

    /**
     * Checks if the roller should be in coast mode
     *
     * @return True if the roller should be in coast mode, false otherwise
     */
    private boolean shouldBeInCoast() {
        return getTopLaserDistance() < 10;
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
     * Checks if the top laser is within the threshold
     *
     * @return True if the top laser is within the threshold, false otherwise
     */
    public boolean topLaserWithinThreshold() {
        return topLaser.withinThreshold();
    }

    /**
     * Checks if the bottom laser is within the threshold
     *
     * @return True if the bottom laser is within the threshold, false otherwise
     */
    public boolean bottomLaserWithinThreshold() {
        return bottomLaser.withinThreshold();
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

        configuration.Commutation.MotorArrangement = Intake.MOTOR_ARRANGEMENT;
        configuration.MotorOutput.Inverted = Intake.INVERTED_VALUE;

        configuration.CurrentLimits.StatorCurrentLimit = Intake.STATOR_CURRENT_LIMIT;
        configuration.CurrentLimits.SupplyCurrentLimit = Intake.SUPPLY_CURRENT_LIMIT;
        configuration.CurrentLimits.SupplyCurrentLowerTime = Intake.SUPPLY_CURRENT_LOWER_TIME;

        configuration.Slot0 = new Slot0Configs().withKP(Intake.KP).withKI(Intake.KI).withKD(Intake.KD);

        configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        return configuration;
    }

    /**
     * Gets the {@link TalonFXSConfiguration} for the follower motor
     *
     * @return The {@link TalonFXSConfiguration} for the follower motor
     */
    private TalonFXSConfiguration followerConfiguration() {
        TalonFXSConfiguration configuration = new TalonFXSConfiguration();

        configuration.Commutation.MotorArrangement = Intake.MOTOR_ARRANGEMENT;
        configuration.MotorOutput.Inverted = Intake.INVERTED_VALUE;

        configuration.CurrentLimits.StatorCurrentLimit = Intake.STATOR_CURRENT_LIMIT;
        configuration.CurrentLimits.SupplyCurrentLimit = Intake.SUPPLY_CURRENT_LIMIT;
        configuration.CurrentLimits.SupplyCurrentLowerTime = Intake.SUPPLY_CURRENT_LOWER_TIME;

        configuration.Slot0 = new Slot0Configs().withKP(Intake.KP).withKI(Intake.KI).withKD(Intake.KD);

        configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        return configuration;
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
