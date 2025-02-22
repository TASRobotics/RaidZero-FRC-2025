package raidzero.robot.subsystems.telescopingarm;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import raidzero.robot.Constants;

public class Intake extends SubsystemBase {
    private static Intake system;

    private TalonFXS roller, follow;

    private LaserCan laserCan;

    /**
     * Constructs a {@link Intake} subsystem instance
     */
    private Intake() {
        roller = new TalonFXS(Constants.TelescopingArm.Intake.MOTOR_ID);
        roller.getConfigurator().apply(rollerConfiguration());

        follow = new TalonFXS(Constants.TelescopingArm.Intake.FOLLOW_ID);
        follow.setControl(new Follower(Constants.TelescopingArm.Intake.MOTOR_ID, true));
        follow.getConfigurator().apply(followConfiguration());

        laserCan = new LaserCan(0);

        try {
            laserCan.setRangingMode(RangingMode.SHORT);
            laserCan.setRegionOfInterest(new RegionOfInterest(8, 4, 6, 8));
            laserCan.setTimingBudget(TimingBudget.TIMING_BUDGET_20MS);
        } catch (Exception e) {
            System.out.println("LaserCan Config Error");
        }
    }

    /**
     * Creates a {@link Command} to run the intake at the specified speed
     * 
     * @param speed The speed as a percentage
     * @return The command to be scheduled and run
     */
    public Command runIntake(double speed) {
        return run(() -> runRoller(speed))
            .until(() -> laserCan.getMeasurement().distance_mm <= 50)
            .andThen(run(() -> runRoller(-0.1)).withTimeout(0.1).andThen(() -> stopRoller()));
    }

    /**
     * Creates a {@link Command} to stop the intake
     * 
     * @return A {@link Command} to stop the intake
     */
    public Command stopRollerCommand() {
        return run(() -> stopRoller());
    }

    /**
     * Creates a {@link Command} to extake at the specified speed
     * 
     * @param speed The desired speed [0, 1.0]
     * @return A {@link Command} to extake at the specified speed
     */
    public Command extake(double speed) {
        return run(() -> runRoller(speed)).withTimeout(1.0).andThen(() -> stopRoller());
    }

    /**
     * Runs the roller at the specified speed
     * 
     * @param speed The speed to run at as a percentage
     */
    private void runRoller(double speed) {
        roller.set(speed);
    }

    /**
     * Stops the roller motor
     */
    public void stopRoller() {
        roller.stopMotor();
    }

    /**
     * Gets the distance from the LaserCAN
     * 
     * @return The distance in mm, -1 if the LaserCAN cannot be found
     */
    public int getLimitDistance() {
        Measurement measurement = laserCan.getMeasurement();

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

        return configuration;
    }

    /**
     * Gets the {@link Intake} subsystem instance
     * 
     * @return The {@link Intake} subsystem instance
     */
    public static Intake system() {
        if (system == null)
            system = new Intake();
        return system;
    }
}
