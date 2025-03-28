package raidzero.robot.subsystems.telescopingarm;

import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidzero.lib.LazyCan;
import raidzero.robot.Constants;
import raidzero.robot.Constants.TelescopingArm.Intake;

public class CoralIntake extends SubsystemBase {
    private TalonFXS roller;

    private LazyCan bottomLaser, topLaser;

    private static CoralIntake system;

    /**
     * Constructs a {@link CoralIntake} subsystem instance
     */
    private CoralIntake() {
        roller = new TalonFXS(Constants.TelescopingArm.Intake.MOTOR_ID, "rio");
        roller.getConfigurator().apply(rollerConfiguration());

        bottomLaser = new LazyCan(1).withRangingMode(RangingMode.SHORT)
            .withRegionOfInterest(8, 8, 16, 16).withTimingBudget(TimingBudget.TIMING_BUDGET_20MS)
            .withThreshold(Intake.BOTTOM_LASER_THRESHOLD_MM);

        topLaser = new LazyCan(0).withRangingMode(RangingMode.SHORT)
            .withRegionOfInterest(8, 8, 16, 16).withTimingBudget(TimingBudget.TIMING_BUDGET_20MS)
            .withThreshold(Intake.TOP_LASER_THRESHOLD_MM);
    }

    /**
     * Sets the motor to static brake mode for disabled init to check for position
     */
    public void enableStaticBrake() {
        roller.setControl(new StaticBrake());
    }

    /**
     * Creates a {@link Command} to run the intake at the specified speed
     *
     * @param speed The speed as a percentage
     * @return The command to be scheduled and run
     */
    public Command intake() {
        return run(() -> roller.set(Intake.INTAKE_SPEED)).until(() -> topLaser.withinThreshold())
            .andThen(
                new SequentialCommandGroup(
                    run(() -> roller.set(Intake.EJECT_SPEED)).onlyIf(() -> isStalling()).withTimeout(0.5),
                    run(() -> roller.set(Intake.INTAKE_LOWER_SPEED)).onlyIf(() -> !isStalling()).until(() -> bottomLaser.withinThreshold())
                        .andThen(() -> roller.set(Intake.REVERSE_SPEED)).until(() -> !bottomLaser.withinThreshold())
                )
            );
    }

    public Command intakeSimple() {
        return run(() -> roller.set(Intake.INTAKE_SPEED)).until(() -> bottomLaser.withinThreshold())
            .andThen(run(() -> roller.set(Intake.REVERSE_SPEED)).withTimeout(0.5));
    }

    /**
     * Returns true if the current is above a pre-defined threshold to consider it stalling
     *
     * @return if the motor is stalliing
     */
    private boolean isStalling() {
        return roller.getStatorCurrent().getValueAsDouble() > Intake.STALL_CURRENT_THRESHOLD;
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
     * Creates a {@link Command} to extake at the specified speed
     *
     * @param speed The desired speed [0, 1.0]
     * @return A {@link Command} to extake at the specified speed
     */
    public Command extake() {
        return run(() -> roller.set(Constants.TelescopingArm.Intake.EXTAKE_SPEED))
            .withTimeout(Constants.TelescopingArm.Intake.EXTAKE_TIMEOUT_S);
    }

    /**
     * Creates a {@link Command} to run the roller at the specified speed
     *
     * @param setpoint The speed to run the roller at [-1, 1]
     * @return A {@link Command} to run the roller at the specified speed
     */
    public Command run(double speed) {
        return run(() -> roller.set(speed));
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

        configuration.Commutation.MotorArrangement = Intake.MOTOR_ARRANGEMENT;
        configuration.MotorOutput.Inverted = Intake.INVERTED_VALUE;

        configuration.CurrentLimits.StatorCurrentLimit = Intake.STATOR_CURRENT_LIMIT;
        configuration.CurrentLimits.SupplyCurrentLimit = Intake.SUPPLY_CURRENT_LIMIT;
        configuration.CurrentLimits.SupplyCurrentLowerTime = Intake.SUPPLY_CURRENT_LOWER_TIME;

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
