package raidzero.robot.subsystems.telescopingarm;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFXS;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import raidzero.robot.Constants;

public class Intake extends SubsystemBase {
    private static Intake system;

    private TalonFXS roller, follow;

    private LaserCan laserCan;

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
     * @param speed the speed as a percentage
     * @return the command to be scheduled and run
     */
    public Command runIntake(double speed) {
        return Commands.run(() -> runRoller(speed), this)
            .until(() -> laserCan.getMeasurement().distance_mm <= 50);
        // .andThen(Commands.run(() -> runRoller(-0.1), this).withTimeout(0.1).andThen(() -> stopRoller()));
    }

    public Command stopRollerCommand() {
        return Commands.run(() -> stopRoller(), this);
    }

    public Command extake(double speed) {
        return Commands.run(() -> runRoller(speed), this).withTimeout(1.0).andThen(() -> stopRoller());
    }

    /**
     * Runs the roller at the specified speed
     * 
     * @param speed the speed to run at as a percentage
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
     * Gets the {@link SparkBaseConfig} for the roller motor
     * 
     * @return the {@link SparkBaseConfig} for the roller motor
     */
    private TalonFXSConfiguration rollerConfiguration() {
        TalonFXSConfiguration configuration = new TalonFXSConfiguration();

        return configuration;
    }

    /**
     * Gets the {@link SparkBaseConfig} for the roller motor
     * 
     * @return the {@link SparkBaseConfig} for the roller motor
     */
    private TalonFXSConfiguration followConfiguration() {
        TalonFXSConfiguration configuration = new TalonFXSConfiguration();

        return configuration;
    }

    public static Intake system() {
        if (system == null)
            system = new Intake();
        return system;
    }
}
