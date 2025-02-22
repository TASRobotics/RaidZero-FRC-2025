package raidzero.robot.subsystems.algaeintake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import raidzero.robot.Constants;

public class Roller extends SubsystemBase {
    private static Roller system;

    private SparkMax roller;

    /**
     * Constructs a {@link Roller} subsystem instance
     */
    private Roller() {
        roller = new SparkMax(Constants.AlgaeIntake.Roller.MOTOR_ID, MotorType.kBrushless);
        roller.configure(rollerConfig(), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /**
     * Runs the roller at the desired speed
     * 
     * @param speed The desired speed
     * @return A {@link Command} that runs the roller at the desired speed
     */
    public Command intake(double speed) {
        return run(() -> run(speed)).withTimeout(2).andThen(() -> stop());
    }

    /**
     * Moves the roller at the desired speed
     * 
     * @param speed The desired speed
     */
    private void run(double speed) {
        roller.set(speed);
    }

    /**
     * Stops the roller motor
     */
    private void stop() {
        roller.stopMotor();
    }

    /**
     * Gets the {@link SparkMaxConfig} for the roller motor
     * 
     * @return The {@link SparkMaxConfig} for the roller motor
     */
    private SparkBaseConfig rollerConfig() {
        SparkMaxConfig configuration = new SparkMaxConfig();

        configuration.idleMode(IdleMode.kBrake);

        return configuration;
    }

    /**
     * Gets the {@link Roller} subsystem instance
     * 
     * @return The {@link Roller} subsystem instance
     */
    public static Roller system() {
        if (system == null) {
            system = new Roller();
        }

        return system;
    }
}