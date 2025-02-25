
package raidzero.robot.subsystems.lighting;

import raidzero.robot.Constants;
import raidzero.robot.subsystems.climb.ClimbJoint;
import raidzero.robot.subsystems.telescopingarm.Arm;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class ArmStrip implements Subsystem {

    private static ArmStrip system;
    
    private CANdle candle;
    private Arm arm;

    private ArmStrip() {
        this.candle = new CANdle(Constants.Lights.CAN_ID,Constants.DRIVETRAIN_CANBUS);
        this.arm = Arm.system();
        configureCandle();
    }

    private void configureCandle() {
        CANdleConfiguration config = new CANdleConfiguration();
        candle.configAllSettings(config);
    }

    public void disabledLEDs() {
        
        if (ClimbJoint.system().getPosition()<0.1){
            if (arm.getJointPosition() >= 0.1675 && arm.getJointPosition() <=0.1726) {
                candle.setLEDs(0, 255, 0); // Green
            } else {
                candle.setLEDs(255, 0, 0); // Red
            }
        } else {
            candle.setLEDs(0, 0, 255); // Blue
        }
        
        
    }

    public static ArmStrip system() {
        if (system == null) {
            system = new ArmStrip();
        }
        return system;
    }
}

