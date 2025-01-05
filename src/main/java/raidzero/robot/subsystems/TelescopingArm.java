package raidzero.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Telescoping arm skeleton code
 */
public class TelescopingArm extends SubsystemBase {
    private static TelescopingArm system;

    private static final double TELESCOPE_CONVERSION_FACTOR = 1.0, ARM_CONVERSION_FACTOR = 1.0;

    private static final int TELESCOPE_MOTOR_ID = 0, ARM_MOTOR_ID = 1, WRIST_MOTOR_ID = 2;
    private static final double TELESCOPE_MAX_LENGTH_M = 1.0 * TELESCOPE_CONVERSION_FACTOR,
            ARM_LENGTH_M = 1.0 * TELESCOPE_CONVERSION_FACTOR;

    private TalonFX telescope, arm, wrist;

    private TelescopingArm() {
        telescope = new TalonFX(TELESCOPE_MOTOR_ID);
        telescope.getConfigurator().apply(telescopeConfiguration());
        telescope.setNeutralMode(NeutralModeValue.Brake);

        arm = new TalonFX(ARM_MOTOR_ID);
        arm.getConfigurator().apply((wristConfiguration()));
        arm.setNeutralMode(NeutralModeValue.Brake);

        wrist = new TalonFX(WRIST_MOTOR_ID);
        wrist.getConfigurator().apply(armConfiguration());
        wrist.setNeutralMode(NeutralModeValue.Brake);
    }

    /**
     * Method to automatically calculate the arm's position and move it to where it needs to be
     * 
     * @param x the x setpoint
     * @param y the y setpoint
     * @return if it is done or not 
     */
    public boolean moveArmTo(double x, double y) {
        final MotionMagicVoltage telescopeRequest = new MotionMagicVoltage(0);
        telescope.setControl(telescopeRequest.withPosition(calculateMinTelescopeHeight(x, y)));

        final MotionMagicVoltage armRequest = new MotionMagicVoltage(0);
        arm.setControl(armRequest.withPosition(calcualteArmAngle(x, y)));

        if (telescope.getPosition().getValueAsDouble() == calculateMinTelescopeHeight(x, y)
                && arm.getPosition().getValueAsDouble() == Math.acos(x / ARM_LENGTH_M) + (Math.PI / 2)) {
            return true;
        } else {
            return false;
        }
    }

    private double calculateMinTelescopeHeight(double x, double y) throws IllegalStateException {
        double height = y - ARM_LENGTH_M * Math.sin(Math.acos(x / ARM_LENGTH_M)) * TELESCOPE_CONVERSION_FACTOR;

        if (height > TELESCOPE_MAX_LENGTH_M)
            throw new IllegalStateException("Arm setpoint too high");

        return height;
    }

    private double calcualteArmAngle(double x, double y) {
        return Math.acos(x / ARM_LENGTH_M) + (Math.PI / 4) * ARM_CONVERSION_FACTOR;
    }

    public static TelescopingArm system() {
        if (system == null) {
            system = new TelescopingArm();
        }
        return system;
    }

    public class MoveArm extends Command {
        private TelescopingArm _arm;
        private double x, y;

        public MoveArm(double x, double y) {
            this._arm = TelescopingArm.system();
            this.x = x;
            this.y = y;

            addRequirements(this._arm);
        }

        @Override
        public void execute() {
            moveArmTo(x, y);
        }

        @Override
        public boolean isFinished() {
            return moveArmTo(TELESCOPE_MOTOR_ID, ARM_MOTOR_ID);
        }

        @Override
        public void end(boolean interrupted) {
            telescope.stopMotor();
            arm.stopMotor();
            wrist.stopMotor();
        }

    }

    private TalonFXConfiguration telescopeConfiguration() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        configuration.Slot0 = new Slot0Configs()
                .withKS(0.25)
                .withKV(0.12)
                .withKA(0.01)
                .withKP(4.8)
                .withKI(0)
                .withKD(0.1);

        configuration.MotionMagic = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(80)
                .withMotionMagicAcceleration(160)
                .withMotionMagicJerk(1600);

        return configuration;
    }

    private TalonFXConfiguration armConfiguration() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        configuration.Slot0 = new Slot0Configs()
                .withKS(0.25)
                .withKV(0.12)
                .withKA(0.01)
                .withKP(4.8)
                .withKI(0)
                .withKD(0.1);

        configuration.MotionMagic = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(80)
                .withMotionMagicAcceleration(160)
                .withMotionMagicJerk(1600);

        return configuration;
    }

    private TalonFXConfiguration wristConfiguration() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        configuration.Slot0 = new Slot0Configs()
                .withKS(0.25)
                .withKV(0.12)
                .withKA(0.01)
                .withKP(4.8)
                .withKI(0)
                .withKD(0.1);

        configuration.MotionMagic = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(80)
                .withMotionMagicAcceleration(160)
                .withMotionMagicJerk(1600);

        return configuration;
    }
}