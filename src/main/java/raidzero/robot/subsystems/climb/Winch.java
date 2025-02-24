package raidzero.robot.subsystems.climb;

import java.lang.constant.Constable;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidzero.robot.Constants;

public class Winch extends SubsystemBase {
    private static Winch system;

    private TalonFX winch;

    private Winch() {
        winch = new TalonFX(Constants.Climb.Winch.MOTOR_ID);
        winch.getConfigurator().apply(winchConfiguration());
        winch.setNeutralMode(NeutralModeValue.Brake);
    }

    public Command runWinch(double speed) {
        return run(() -> winch.set(speed));
    }

    public Command stopMotor() {
        return run(() -> winch.stopMotor());
    }

    private TalonFXConfiguration winchConfiguration() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        return configuration;
    }

    public static Winch system() {
        if (system == null) {
            system = new Winch();
        }
        return system;
    }
}
