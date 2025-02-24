package raidzero.robot.subsystems.telescopingarm;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import raidzero.robot.Constants;

public class CoralIntake extends SubsystemBase {
    private TalonFXS roller, follow;

    private LaserCan bottomLaser, topLaser;

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

        bottomLaser = new LaserCan(0);
        try {
            bottomLaser.setRangingMode(RangingMode.SHORT);
            bottomLaser.setRegionOfInterest(new RegionOfInterest(8, 4, 6, 8));
            bottomLaser.setTimingBudget(TimingBudget.TIMING_BUDGET_20MS);
        } catch (Exception e) {
            System.out.println("LaserCan Config Error");
        }

        topLaser = new LaserCan(1);
        try {
            topLaser.setRangingMode(RangingMode.SHORT);
            topLaser.setRegionOfInterest(new RegionOfInterest(8, 4, 6, 8));
            topLaser.setTimingBudget(TimingBudget.TIMING_BUDGET_20MS);
        } catch (Exception e) {
            System.out.println("LaserCan Config Error");
        }
    }
    /**
     * Gets the roller motor controller for disabled init to check for position
     * @return The Roller motor
     */
    public TalonFXS getRoller() {
        return roller;
    }

    /**
     * Creates a {@link Command} to run the intake at the specified speed
     * 
     * @param speed The speed as a percentage
     * @return The command to be scheduled and run
     */
    public Command intake() {
        return run(() -> roller.set(Constants.TelescopingArm.Intake.INTAKE_SPEED))
            .until(() -> bottomLaser.getMeasurement().distance_mm <= Constants.TelescopingArm.Intake.LASERCAN_DISTANCE_THRESHOLD_MM)
            .andThen(
                run(() -> roller.set(-Constants.TelescopingArm.Intake.INTAKE_SPEED))
                    .withTimeout(0.1)
            );
    }

    public Command scoochCoral() {
        return run(() -> roller.set(-Constants.TelescopingArm.Intake.INTAKE_SPEED))
            .until(() -> topLaser.getMeasurement().distance_mm <= Constants.TelescopingArm.Intake.LASERCAN_DISTANCE_THRESHOLD_MM);
    }

    /**
     * Creates a {@link Command} to stop the intake
     * 
     * @return A {@link Command} to stop the intake
     */
    public Command stopRoller() {
        return run(() -> roller.stopMotor());
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
        Measurement measurement = topLaser.getMeasurement();

        return measurement != null ? measurement.distance_mm : -1;
    }

    /**
     * Gets the distance from the LaserCAN
     * 
     * @return The distance in mm, -1 if the LaserCAN cannot be found
     */
    public int getBottomLaserDistance() {
        Measurement measurement = bottomLaser.getMeasurement();

        return measurement != null ? measurement.distance_mm : -1;
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
