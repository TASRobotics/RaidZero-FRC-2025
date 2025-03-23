package raidzero.robot.subsystems.climb;

import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidzero.robot.Constants;

public class Winch extends SubsystemBase {
    private TalonFX winch;

    private static Winch system;

    /**
     * Constructs a {@link Winch} subsystem instance
     */
    private Winch() {
        winch = new TalonFX(Constants.Climb.Winch.MOTOR_ID);
        winch.getConfigurator().apply(winchConfiguration());
        winch.setNeutralMode(NeutralModeValue.Brake);
    }

    /**
     * Runs the winch at the specified speed
     *
     * @param speed The speed to run the winch at
     * @return A {@link Command} that runs the winch at the specified speed
     */
    public Command run(double speed, boolean ramp) {
        if(ramp)
            winch.getConfigurator().apply(winchConfigWithRampRate());

        return run(() -> winch.set(speed));
    }

    /**
     * Stops the winch motor
     *
     * @return A {@link Command} that stops the winch motor
     */
    public Command stop() {
        return run(() -> winch.stopMotor());
    }

    /**
     * Returns a {@link TalonFXConfiguration} for the winch
     *
     * @return A {@link TalonFXConfiguration} for the winch
     */
    private TalonFXConfiguration winchConfiguration() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        return configuration;
    }

    private TalonFXConfiguration winchConfigWithRampRate() {
        TalonFXConfiguration configuration = new TalonFXConfiguration()
            .withOpenLoopRamps(new OpenLoopRampsConfigs()
                .withDutyCycleOpenLoopRampPeriod(0.2));

        return configuration;
    }

    /**
     * Returns the {@link Winch} subsystem instance
     *
     * @return The {@link Winch} subsystem instance
     */
    public static Winch system() {
        if (system == null) {
            system = new Winch();
        }

        return system;
    }
}
