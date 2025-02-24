
package raidzero.robot.subsystems.lighting;

import raidzero.robot.Constants;
import raidzero.robot.subsystems.telescopingarm.Arm;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class ArmStrip implements Subsystem {

    private static ArmStrip system;
    
    private CANdle candle;
    private Arm arm;

    private ArmStrip() {
        this.candle = new CANdle(Constants.Lights.CAN_ID);
        this.arm = Arm.system();
        configureCandle();
    }

    private void configureCandle() {
        CANdleConfiguration config = new CANdleConfiguration();
        candle.configAllSettings(config);
    }

    public void disabledLEDs() {
        
        if (arm.getJointPosition() >= 0.14 && arm.getJointPosition() <=0.16) {
            candle.setLEDs(0, 255, 0); // Green
        } else {
            candle.setLEDs(255, 0, 0); // Red
        }
        
        
    }

    public static ArmStrip system() {
        if (system == null) {
            system = new ArmStrip();
        }
        return system;
    }
}

