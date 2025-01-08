package raidzero.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidzero.robot.Constants;
import raidzero.robot.wrappers.LimelightHelpers;

public class Limelight extends SubsystemBase {
    public enum LED_MODE {
        PIPELINE,
        OFF,
        BLINK,
        ON
    }

    public enum STREAM_MODE {
        STANDARD,
        PIP_MAIN,
        PIP_SECOND
    }

    private String objectLimelightName = Constants.Limelight.OBJECT_LIMELIGHT_NAME;

    private static Limelight instance = null;

    private Limelight() {}

    public void setStreamMode(String limelightName, STREAM_MODE mode) {
        if (mode == STREAM_MODE.STANDARD) {
            LimelightHelpers.setStreamMode_Standard(limelightName);
        } else if (mode == STREAM_MODE.PIP_MAIN) {
            LimelightHelpers.setStreamMode_PiPMain(limelightName);
        } else if (mode == STREAM_MODE.PIP_SECOND) {
            LimelightHelpers.setStreamMode_PiPSecondary(limelightName);
        }
    }

    public void setPipeline(String limelightName, int pipeline) {
        LimelightHelpers.setPipelineIndex(limelightName, pipeline);
    }

    public void setLedMode(String limelightName, LED_MODE mode) {
        if (mode == LED_MODE.PIPELINE) {
            LimelightHelpers.setLEDMode_PipelineControl(limelightName);
        } else if (mode == LED_MODE.OFF) {
            LimelightHelpers.setLEDMode_ForceOff(limelightName);
        } else if (mode == LED_MODE.BLINK) {
            LimelightHelpers.setLEDMode_ForceBlink(limelightName);
        } else if (mode == LED_MODE.ON) {
            LimelightHelpers.setLEDMode_ForceOn(limelightName);
        }
    }

    public void initialize() {
        LimelightHelpers.setCameraPose_RobotSpace(
            "limelight-left",
            Constants.Limelight.Positions.LEFT_X,
            Constants.Limelight.Positions.LEFT_Z,
            Constants.Limelight.Positions.LEFT_Y,
            Constants.Limelight.Positions.LEFT_ROLL,
            Constants.Limelight.Offsets.LEFT_PITCH,
            Constants.Limelight.Offsets.LEFT_YAW
        );

        LimelightHelpers.setCameraPose_RobotSpace(
            "limelight-right",
            Constants.Limelight.Positions.RIGHT_X,
            Constants.Limelight.Positions.RIGHT_Z,
            Constants.Limelight.Positions.RIGHT_Y,
            Constants.Limelight.Positions.RIGHT_ROLL,
            Constants.Limelight.Offsets.RIGHT_PITCH,
            Constants.Limelight.Offsets.RIGHT_YAW
        );

        LimelightHelpers.setCameraPose_RobotSpace(
            "limelight-back",
            Constants.Limelight.Positions.BACK_Z,
            Constants.Limelight.Positions.BACK_X,
            Constants.Limelight.Positions.BACK_Y,
            Constants.Limelight.Positions.BACK_ROLL,
            Constants.Limelight.Offsets.BACK_PITCH,
            Constants.Limelight.Offsets.BACK_YAW
        );
    }

    public static Limelight system() {
        if (instance == null) {
            instance = new Limelight();
        }

        return instance;
    }
}