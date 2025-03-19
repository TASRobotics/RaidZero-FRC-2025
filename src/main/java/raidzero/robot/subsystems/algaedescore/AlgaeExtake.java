package raidzero.robot.subsystems.algaedescore;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidzero.robot.Constants;

public class AlgaeExtake extends SubsystemBase {
    private SparkMax roller;

    private static AlgaeExtake system;

    /**
     * Constructs a {@link AlgaeExtake} subsystem instance
     */
    private AlgaeExtake() {
        roller = new SparkMax(Constants.AlgaeDescore.Extake.MOTOR_ID, MotorType.kBrushless);
        roller.configure(rollerConfig(), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /**
     * Runs the roller at the desired speed
     *
     * @param speed The desired speed
     * @return A {@link Command} that runs the roller at the desired speed
     */
    public Command extake(double speed) {
        return run(() -> run(speed));
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
    public Command stop() {
        return run(() -> roller.stopMotor());
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
     * Gets the {@link AlgaeExtake} subsystem instance
     *
     * @return The {@link AlgaeExtake} subsystem instance
     */
    public static AlgaeExtake system() {
        if (system == null) {
            system = new AlgaeExtake();
        }

        return system;
    }
}