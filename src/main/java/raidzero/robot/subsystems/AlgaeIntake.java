package raidzero.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {
    private static AlgaeIntake system;

    private TalonFX joint;
    private TalonFX climb;
    private SparkMax roller;

    private AlgaeIntake() {
        joint = new TalonFX(0);
        joint.getConfigurator().apply(jointConfiguration());
        joint.setNeutralMode(NeutralModeValue.Brake);

        climb = new TalonFX(0);
        climb.getConfigurator().apply(climbConfiguration());
        climb.setNeutralMode(NeutralModeValue.Brake);

    }

    private TalonFXConfiguration jointConfiguration() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();



        return configuration;
    }

    private TalonFXConfiguration climbConfiguration() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();
        return configuration;
    }

    private  SparkBaseConfig rollerConfig() {
        SparkMaxConfig configuration = new SparkMaxConfig();
        return configuration;
    }

    public static AlgaeIntake system() {
        if (system == null)
            system = new AlgaeIntake();
        return system;
    }
}