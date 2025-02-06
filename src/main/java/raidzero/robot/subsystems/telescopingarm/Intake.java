package raidzero.robot.subsystems.telescopingarm;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import raidzero.robot.subsystems.telescopingarm.Constants.Roller;

public class Intake extends SubsystemBase {
    private static Intake system;

    private SparkMax roller, rollerFollow;

    private LaserCan limit;

    private Intake() {
        roller = new SparkMax(Roller.MOTOR_ID, MotorType.kBrushless);
        roller.configure(rollerConfiguration(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rollerFollow = new SparkMax(Roller.FOLLOW_ID, MotorType.kBrushless);
        rollerFollow.configure(rollerFollowConfiguration(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        limit = new LaserCan(0);

        try {
            limit.setRangingMode(RangingMode.SHORT);
            limit.setRegionOfInterest(new RegionOfInterest(8, 4, 6, 8));
            limit.setTimingBudget(TimingBudget.TIMING_BUDGET_20MS);
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
            .until(() -> limit.getMeasurement().distance_mm <= 40)
            .andThen(Commands.run(() -> runRoller(-0.1), this).withTimeout(0.2).andThen(() -> stopRoller()));
    }

    public Command extake(double speed) {
        return Commands.run(() -> runRoller(-speed), this).withTimeout(1.0).andThen(() -> stopRoller());
    }

    /**
     * Runs the roller at the specified speed
     * 
     * @param speed the speed to run at as a percentage
     */
    private void runRoller(double speed) {
        roller.set(-speed);
    }

    /**
     * Stops the roller motor
     */
    public void stopRoller() {
        roller.stopMotor();
    }

    /**
     * Gets the {@link SparkBaseConfig} for the roller motor
     * 
     * @return the {@link SparkBaseConfig} for the roller motor
     */
    private SparkBaseConfig rollerConfiguration() {
        SparkMaxConfig configuration = new SparkMaxConfig();

        configuration.idleMode(IdleMode.kBrake);

        return configuration;
    }

    /**
     * Gets the {@link SparkBaseConfig} for the roller follow motor
     * 
     * @return the {@link SparkBaseConfig} for the roller follow motor
     */
    private SparkBaseConfig rollerFollowConfiguration() {
        SparkMaxConfig configuration = new SparkMaxConfig();

        configuration.follow(Roller.MOTOR_ID, true);
        configuration.idleMode(IdleMode.kBrake);

        return configuration;
    }

    public static Intake system() {
        if (system == null)
            system = new Intake();
        return system;
    }
}
