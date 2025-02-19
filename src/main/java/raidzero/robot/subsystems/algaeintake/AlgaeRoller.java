package raidzero.robot.subsystems.algaeintake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import raidzero.robot.Constants;

public class AlgaeRoller extends SubsystemBase {
    private static AlgaeRoller system;

    private SparkMax roller;

    private AlgaeRoller() {
        roller = new SparkMax(Constants.AlgaeIntake.Roller.MOTOR_ID, MotorType.kBrushless);
        roller.configure(rollerConfig(), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public Command runRoller(double speed) {
        return Commands.run(() -> moveRoller(speed)).withTimeout(2).andThen(() -> stopRoller());
    }

    private void moveRoller(double speed) {
        roller.set(speed);
    }

    public Command stopRoller() {
        return Commands.run(() -> roller.stopMotor(), this);
    }

    private SparkBaseConfig rollerConfig() {
        SparkMaxConfig configuration = new SparkMaxConfig();

        configuration.idleMode(IdleMode.kBrake);

        return configuration;
    }

    public static AlgaeRoller system() {
        if (system == null)
            system = new AlgaeRoller();
        return system;
    }
}