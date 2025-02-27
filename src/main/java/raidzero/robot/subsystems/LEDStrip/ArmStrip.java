
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
    private boolean test = false;
    private boolean rainbow = false;

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
     * The main loop of the CANdle LED strip
     */
    public void loop() {
        armIsLegal = arm.getJointPosition() >= Constants.CANdle.ARM_JOINT_LOWER_BOUND &&
            arm.getJointPosition() <= Constants.CANdle.ARM_JOINT_UPPER_BOUND;

        if (DriverStation.isDisabled()) {
            if (!armIsLegal && ClimbJoint.system().getPosition() > 0.1) {
                    candle.setLEDs(255, 0, 0);
                    candle.clearAnimation(0);
                    test = false;
            } else if (ClimbJoint.system().getPosition() < 0.1 && !armIsLegal && !ClimbJoint.system().isDeployed().getAsBoolean()) {
                if (!test) {
                    candle.animate(new StrobeAnimation(255, 0, 0, 0, 0.05, -1));
                    test = true;
                }
            } else if (armIsLegal) {
                    candle.setLEDs(0, 255, 0);
                    candle.clearAnimation(0);
                    test = false;
            }
        } else if (DriverStation.isAutonomousEnabled()) {
            if (!test) {
                candle.animate(new StrobeAnimation(255, 165, 0, 0, 0.0001, 33), 0);
                candle.animate(new StrobeAnimation(0, 255, 0, 0, 0.0001, 28, 33), 1);
                test = true;
            }
        } else if (DriverStation.isAutonomous() && !DriverStation.isAutonomousEnabled()) {
            candle.clearAnimation(0);
            candle.clearAnimation(1);
        } else if (DriverStation.isTeleop() && !DriverStation.isTeleopEnabled()) {
            candle.clearAnimation(0);
            candle.clearAnimation(1);
        } else if (DriverStation.isTeleopEnabled()) {
            if (ClimbJoint.system().isDeployed().getAsBoolean()) {
                candle.animate(new StrobeAnimation(0, 0, 255, 0, 0.05, -1));
            } else {
                if (!rainbow) {
                    candle.clearAnimation(0);
                    candle.clearAnimation(1);
                    candle.animate(new RainbowAnimation(255, 0.75, -1));
                    rainbow = true;
                }
            }
        } else if (DriverStation.isTestEnabled()) {
            candle.clearAnimation(0);
            candle.clearAnimation(1);
        }
    }

    /**
     * Clears CAndle animaitons
     */
    public void clearAnimation() {
        test = false;
        rainbow = false;
        candle.clearAnimation(0);
        candle.clearAnimation(1);
    }
    
    /**
     * Plays the match end animation
     */
    public void endAnimation() {
        candle.clearAnimation(0);
        candle.clearAnimation(1);
        candle.animate(new RainbowAnimation(255, 0.75, -1));
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
