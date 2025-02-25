
package raidzero.robot.subsystems.LEDStrip;

import raidzero.robot.Constants;
import raidzero.robot.subsystems.climb.ClimbJoint;
import raidzero.robot.subsystems.telescopingarm.Arm;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class ArmStrip implements Subsystem {

    private CANdle candle;
    private Arm arm;

    private static ArmStrip system;

    /**
     * Constructs a {@link ArmStrip} subsystem.
     */
    private ArmStrip() {
        this.candle = new CANdle(Constants.CANdle.CAN_ID, Constants.CANIVORE_NAME);
        this.arm = Arm.system();

        this.candle.configAllSettings(candleConfig());
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
