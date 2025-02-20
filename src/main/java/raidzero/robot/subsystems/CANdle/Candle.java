package raidzero.robot.subsystems.CANdle;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidzero.robot.Constants;
import raidzero.robot.subsystems.drivetrain.Swerve;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

public class Candle extends SubsystemBase {
    private final CANdle candle;
    private Swerve swerve = Swerve.system();

    /**
     * Constructor for the CANdle subsystem
     */
    private Candle() {
        candle = new CANdle(Constants.CANdle.CANdleID);
        candle.setLEDs(0, 0, 0);
        candle.configAllSettings(getCANdleConfiguration());

    }

    /**
     * Sets animation mode of the CANdle led
     * 
     * @param r Red value of the flow [0, 255]
     * @param g Green value of the flow [0, 255]
     * @param b Blue value of the flow [0, 255]
     * @param w White value of the flow [0, 255]
     * @param speed Speed value of the flow [0.0, 1.0]
     */
    public void setAnimationMode(int r, int g, int b, int w, double speed, Constants.CANdle.MODES mode) {
        switch (mode) {
            case COLOR_FLOW:
                candle.animate(new ColorFlowAnimation(r, g, b, w, speed, Constants.CANdle.ledCount, Direction.Backward));
                break;
            case STROBE:
                candle.animate(new StrobeAnimation(r, g, b, w, speed, Constants.CANdle.ledCount));
                break;
            case RAINDBOW:
                candle.animate(new RainbowAnimation(0.5, speed, Constants.CANdle.ledCount));
                break;
            case LARSON:
                candle.animate(new LarsonAnimation(r, g, b, w, speed, Constants.CANdle.ledCount, LarsonAnimation.BounceMode.Front, 4));
                break;
        }
    }

    /**
     * Sets the color of the CANdle to the desired color
     * 
     * @param color {@link Constants.CANdle.COLORS} enum
     */
    public void setColor(Constants.CANdle.COLORS color) {
        switch (color) {
            case RED:
                candle.setLEDs(255, 0, 0);
                break;
            case GREEN:
                candle.setLEDs(0, 204, 14);
                break;
            case YELLOW:
                candle.setLEDs(247, 243, 0);
                break;
            case ORANGE:
                candle.setLEDs(247, 147, 0);
                break;
            default:
                candle.setLEDs(0, 0, 0);
                break;
        }
    }

    @Override
    public void periodic() {
        AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        boolean intakeReady = false;
        intakeReady = (layout.getTagPose(Constants.CANdle.stationIds[0]).get().toPose2d().getTranslation()
            .getDistance(swerve.getState().Pose.getTranslation()) < Constants.CANdle.distThreshold) ||
            (layout.getTagPose(Constants.CANdle.stationIds[1]).get().toPose2d().getTranslation()
                .getDistance(swerve.getState().Pose.getTranslation()) < Constants.CANdle.distThreshold) ||
            (layout.getTagPose(Constants.CANdle.stationIds[2]).get().toPose2d().getTranslation()
                .getDistance(swerve.getState().Pose.getTranslation()) < Constants.CANdle.distThreshold) ||
            (layout.getTagPose(Constants.CANdle.stationIds[3]).get().toPose2d().getTranslation()
                .getDistance(swerve.getState().Pose.getTranslation()) < Constants.CANdle.distThreshold);
        if (intakeReady) {
            this.setAnimationMode(247, 147, 0, 0, 0.5, Constants.CANdle.MODES.LARSON);
        }
        
    }

    /**
     * Creates a {@link CANdleConfiguration} for CANdle
     */
    private CANdleConfiguration getCANdleConfiguration() {
        com.ctre.phoenix.led.CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB;
        config.brightnessScalar = 0.5;

        return config;
    }
}