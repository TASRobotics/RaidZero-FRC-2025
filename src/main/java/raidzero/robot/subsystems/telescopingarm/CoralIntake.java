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
        roller = new TalonFXS(Constants.TelescopingArm.Intake.MOTOR_ID, "rio");
        roller.getConfigurator().apply(rollerConfiguration());

        follower = new TalonFXS(13);
        follower.getConfigurator().apply(followerConfiguration());
        follower.setControl(new Follower(Intake.MOTOR_ID, false));

        bottomLaser = new LazyCan(1).withRangingMode(RangingMode.SHORT)
            .withRegionOfInterest(14, 8, 4, 16).withTimingBudget(TimingBudget.TIMING_BUDGET_20MS)
            .withThreshold(Intake.BOTTOM_LASER_THRESHOLD_MM);

        topLaser = new LazyCan(0).withRangingMode(RangingMode.LONG)
            .withRegionOfInterest(8, 14, 16, 4).withTimingBudget(TimingBudget.TIMING_BUDGET_20MS)
            .withThreshold(Intake.TOP_LASER_THRESHOLD_MM);
    }

    /**
     * Intakes coral
     *
     * @return A {@link Command}
     */
    public Command intake() {
        return run(() -> roller.set(Intake.INTAKE_SPEED)).until(() -> bottomLaser.withinThreshold());
    }

    /**
     * Intakes Algae

     * @return A {@link Command}
     */
    public Command intakeAlgae() {
        return run(() -> roller.set(Intake.INTAKE_SPEED));
    }

    /**
     * Extakes Algae
     *
     * @return A {@link Command}
     */
    public Command extaxeAlgae() {
        return run(() -> roller.set(Intake.ALGAE_EJECT_SPEED));
    }

    /**
     * Holds the algae by applying a small amount of voltage
     *
     * @return A {@link Command}
     */
    public Command holdAlgae() {
        return run(() -> roller.set(Intake.HOLD_SPEED));
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
     * @return A {@link Command} to extake at the specified speed
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

    /**
     * Gets the distance from the LaserCAN
     *
     * @return The distance in mm, -1 if the LaserCAN cannot be found
     */
    public int getTopLaserDistance() {
        return topLaser.getDistanceMm();
    }

    public boolean topLaserWithinThreshold() {
        return topLaser.withinThreshold();
    }

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
