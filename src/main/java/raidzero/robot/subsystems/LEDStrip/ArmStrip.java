
package raidzero.robot.subsystems.LEDStrip;

import raidzero.robot.Constants;
import raidzero.robot.subsystems.climb.ClimbJoint;
import raidzero.robot.subsystems.telescopingarm.Arm;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ArmStrip implements Subsystem {

    private CANdle candle;
    private Arm arm;

    private boolean armIsLegal = false;

    private int alternateLed = 0;

    private Notifier notifier;
    
    private static ArmStrip system;

    /**
     * Constructs a {@link ArmStrip} subsystem.
     */
    private ArmStrip() {
        this.candle = new CANdle(Constants.CANdle.CAN_ID, Constants.CANIVORE_NAME);
        this.arm = Arm.system();

        this.candle.configAllSettings(candleConfig());

        this.notifier = new Notifier(this::loop);
        notifier.startPeriodic(0.2);
    }

    /**
     * Sets the LEDs to the correct color based on the state of the robot.
     */
    public void disabledLEDs() {
        if (ClimbJoint.system().getPosition() < 0.1) {
            if (arm.getJointPosition() >= Constants.CANdle.ARM_JOINT_LOWER_BOUND &&
                arm.getJointPosition() <= Constants.CANdle.ARM_JOINT_UPPER_BOUND) {
                candle.setLEDs(0, 255, 0);
            } else {
                candle.setLEDs(255, 0, 0);
            }
        } else {
            candle.setLEDs(0, 0, 255);
        }
    }

    public void loop() {
        armIsLegal = arm.getJointPosition() >= Constants.CANdle.ARM_JOINT_LOWER_BOUND &&
            arm.getJointPosition() <= Constants.CANdle.ARM_JOINT_UPPER_BOUND;

        if (DriverStation.isDisabled()) {
            if (!armIsLegal) {
                candle.setLEDs(255, 0, 0);
            } else if (ClimbJoint.system().getPosition() < 0.1 && !armIsLegal) {
                candle.animate(new StrobeAnimation(255, 0, 0, 0, 0.5, -1));
            } else if (armIsLegal) {
                candle.setLEDs(0, 255, 0);
            }
        } else if (DriverStation.isAutonomousEnabled()) {
            if (alternateLed == 0) {
                candle.setLEDs(255, 165, 0);
                alternateLed = 1;
            } else {
                candle.setLEDs(0, 255, 0);
                alternateLed = 0;
            }
        } else if (DriverStation.isTeleopEnabled()) {
            if (ClimbJoint.system().isDeployed().getAsBoolean()) {
                candle.animate(new StrobeAnimation(0, 0, 255, 0, 0.5, -1));
            } else {
                candle.animate(new RainbowAnimation(255, 0.5, -1));
            }
        }
    }

    /**
     * Gets a {@link CANdleConfiguration} for the CANdle LED strip
     * 
     * @return A {@link CANdleConfiguration} for the CANdle LED strip
     */
    private CANdleConfiguration candleConfig() {
        return new CANdleConfiguration();
    }

    /**
     * Gets the {@link ArmStrip} subsystem instance
     * 
     * @return The {@link ArmStrip} subsystem instance
     */
    public static ArmStrip system() {
        if (system == null) {
            system = new ArmStrip();
        }
        return system;
    }
}
