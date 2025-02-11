package raidzero.robot.subsystems.drivetrain;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidzero.robot.Constants;
import raidzero.robot.wrappers.LimelightHelpers;

public class Limelight extends SubsystemBase {
    public enum LED_MODE {
        PIPELINE, OFF, BLINK, ON
    }

    public enum STREAM_MODE {
        STANDARD, PIP_MAIN, PIP_SECOND
    }

    private boolean ignoreFlLime = false;
    private boolean ignoreFrLime = false;
    private boolean ignoreBlLime = false;
    private boolean ignoreBrLime = false;
    private boolean ignoreAllLimes = false;
    
    private StructPublisher <Pose2d> flNT = NetworkTableInstance.getDefault().getStructTopic("flNT", Pose2d.struct).publish();
    private StructPublisher <Pose2d> frNT = NetworkTableInstance.getDefault().getStructTopic("frNT", Pose2d.struct).publish();
    private StructPublisher <Pose2d> blNT = NetworkTableInstance.getDefault().getStructTopic("blNT", Pose2d.struct).publish();
    private StructPublisher <Pose2d> brNT = NetworkTableInstance.getDefault().getStructTopic("brNT", Pose2d.struct).publish();

    private LimelightHelpers.PoseEstimate limeFL, limeFR, limeBL, limeBR;
    private LimelightHelpers.PoseEstimate limeFLPrev, limeFRPrev, limeBLPrev, limeBRPrev;

    private Swerve swerve = Swerve.system();
    private static Limelight instance = null;

    private Limelight() {
        // initialize();
    }

    public void setStreamMode(String limelightName, STREAM_MODE mode) {
        if (mode == STREAM_MODE.STANDARD) {
            LimelightHelpers.setStreamMode_Standard(limelightName);
        } else if (mode == STREAM_MODE.PIP_MAIN) {
            LimelightHelpers.setStreamMode_PiPMain(limelightName);
        } else if (mode == STREAM_MODE.PIP_SECOND) {
            LimelightHelpers.setStreamMode_PiPSecondary(limelightName);
        }
    }

    /**
     * Sets the pipeline of the limelight
     * 
     * @param limelightName The name of the limelight
     * @param pipeline The pipeline index
     */
    public void setPipeline(String limelightName, int pipeline) {
        LimelightHelpers.setPipelineIndex(limelightName, pipeline);
    }

    /**
     * Sets the LED mode of the limelight
     * 
     * @param limelightName The name of the limelight
     * @param mode The LED mode
     */
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

    @Override
    public void periodic() {
        if (swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble() > 720) {
            ignoreAllLimes = true;
        } else {
            ignoreAllLimes = false;
        }

        LimelightHelpers.SetRobotOrientation(
            "limelight-fl",
            swerve.getState().Pose.getRotation().getDegrees(),
            swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble(),
            0,
            0,
            0,
            0
        );
        limeFL = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-fl");

        if (limeFL != null && limeFL.pose != null) {
            ignoreFlLime = !poseInField(limeFL.pose) ||
                (Math.abs(LimelightHelpers.getBotPose3d_wpiBlue("limelight-fl").getZ()) > 0.4) ||
                (LimelightHelpers.getTA("limelight-fl") < 0.1) ||
                (limeFLPrev != null && (limeFL.pose.getTranslation().getDistance(limeFLPrev.pose.getTranslation()) /
                    (limeFL.timestampSeconds - limeFLPrev.timestampSeconds)) > TunerConstants.kSpeedAt12Volts.baseUnitMagnitude()) ||
                (limeFL.rawFiducials.length > 0 && limeFL.rawFiducials[0].ambiguity > 0.5 &&
                    limeFL.rawFiducials[0].distToCamera > 3.5);

            if (!ignoreAllLimes && !ignoreFlLime) {
                SmartDashboard.putBoolean("FLpose", true);
                flNT.set(limeFL.pose);

                swerve.addVisionMeasurement(
                    limeFL.pose,
                    Utils.fpgaToCurrentTime(limeFL.timestampSeconds),
                    VecBuilder.fill(0.5, 0.5, 9999999).div(LimelightHelpers.getTA("limelight-fl"))
                );
            } else {
                SmartDashboard.putBoolean("FLpose", false);
            }

            limeFLPrev = limeFL;
        }

        LimelightHelpers.SetRobotOrientation(
            "limelight-fr",
            swerve.getState().Pose.getRotation().getDegrees(),
            swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble(),
            0,
            0,
            0,
            0
        );
        limeFR = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-fr");

        if (limeFR != null && limeFR.pose != null) {
            ignoreFrLime = !poseInField(limeFR.pose) ||
                (Math.abs(LimelightHelpers.getBotPose3d_wpiBlue("limelight-fr").getZ()) > 0.4) ||
                (LimelightHelpers.getTA("limelight-fr") < 0.1) ||
                (limeFRPrev != null && (limeFR.pose.getTranslation().getDistance(limeFRPrev.pose.getTranslation()) /
                    (limeFR.timestampSeconds - limeFRPrev.timestampSeconds)) > TunerConstants.kSpeedAt12Volts.baseUnitMagnitude()) ||
                (limeFR.rawFiducials.length > 0 && limeFR.rawFiducials[0].ambiguity > 0.5 &&
                    limeFR.rawFiducials[0].distToCamera > 3.5);

            if (!ignoreAllLimes && !ignoreFrLime) {
                SmartDashboard.putBoolean("FRpose", true);
                frNT.set(limeFR.pose);

                swerve.addVisionMeasurement(
                    limeFR.pose,
                    Utils.fpgaToCurrentTime(limeFR.timestampSeconds),
                    VecBuilder.fill(0.5, 0.5, 9999999).div(LimelightHelpers.getTA("limelight-fr"))
                );
            } else {
                SmartDashboard.putBoolean("FRpose", false);
            }

            limeFRPrev = limeFR;
        }

        LimelightHelpers.SetRobotOrientation(
            "limelight-bl",
            swerve.getState().Pose.getRotation().getDegrees(),
            swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble(),
            0,
            0,
            0,
            0
        );
        limeBL = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-bl");

        if (limeBL != null && limeBL.pose != null) {
            ignoreBlLime = !poseInField(limeBL.pose) ||
                (Math.abs(LimelightHelpers.getBotPose3d_wpiBlue("limelight-bl").getZ()) > 0.4) ||
                (LimelightHelpers.getTA("limelight-bl") < 0.1) ||
                (limeBLPrev != null && (limeBL.pose.getTranslation().getDistance(limeBLPrev.pose.getTranslation()) /
                    (limeBL.timestampSeconds - limeBLPrev.timestampSeconds)) > TunerConstants.kSpeedAt12Volts.baseUnitMagnitude()) ||
                (limeBL.rawFiducials.length > 0 && limeBL.rawFiducials[0].ambiguity > 0.5 &&
                    limeBL.rawFiducials[0].distToCamera > 3.5);

            if (!ignoreAllLimes && !ignoreBlLime) {
                SmartDashboard.putBoolean("BLpose", true);
                blNT.set(limeBL.pose);

                swerve.addVisionMeasurement(
                    limeBL.pose,
                    Utils.fpgaToCurrentTime(limeBL.timestampSeconds),
                    VecBuilder.fill(0.75, 0.75, 9999999).div(LimelightHelpers.getTA("limelight-bl"))
                );
            } else {
                SmartDashboard.putBoolean("BLpose", false);
            }

            limeBLPrev = limeBL;
        }

        LimelightHelpers.SetRobotOrientation(
            "limelight-br",
            swerve.getState().Pose.getRotation().getDegrees(),
            swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble(),
            0,
            0,
            0,
            0
        );
        limeBR = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-br");

        if (limeBR != null && limeBR.pose != null) {
            ignoreBrLime = !poseInField(limeBR.pose) ||
                (Math.abs(LimelightHelpers.getBotPose3d_wpiBlue("limelight-br").getZ()) > 0.4) ||
                (LimelightHelpers.getTA("limelight-br") < 0.1) ||
                (limeBRPrev != null && (limeBR.pose.getTranslation().getDistance(limeBRPrev.pose.getTranslation()) /
                    (limeBR.timestampSeconds - limeBRPrev.timestampSeconds)) > TunerConstants.kSpeedAt12Volts.baseUnitMagnitude()) ||
                (limeBR.rawFiducials.length > 0 && limeBR.rawFiducials[0].ambiguity > 0.5 &&
                    limeBR.rawFiducials[0].distToCamera > 3.5);

            if (!ignoreAllLimes && !ignoreBrLime) {
                SmartDashboard.putBoolean("BRpose", true);
                brNT.set(limeBR.pose);

                swerve.addVisionMeasurement(
                    limeBR.pose,
                    Utils.fpgaToCurrentTime(limeBR.timestampSeconds),
                    VecBuilder.fill(0.75, 0.75, 9999999).div(LimelightHelpers.getTA("limelight-br"))
                );
            } else {
                SmartDashboard.putBoolean("BRpose", false);
            }

            limeBRPrev = limeBR;
        }
    }

    /**
     * Checks if a pose is inside the field dimensions
     * 
     * @param pose The {@link Pose2d} to check
     * @return True if the pose is inside the field dimensions, false otherwise
     */
    private boolean poseInField(Pose2d pose) {
        return pose.getTranslation().getX() < 16 && pose.getTranslation().getY() < 8;
    }

    /**
     * <ul><li>Initializes the limelight subsystem
     * <li>Configures the limelight camera poses</ul>
     */
    private void initialize() {
        LimelightHelpers.setCameraPose_RobotSpace(
            "limelight-fl",
            Constants.Limelight.Offsets.FL_X_OFFSET,
            Constants.Limelight.Offsets.FL_Z_OFFSET,
            Constants.Limelight.Offsets.FL_Y_OFFSET,
            Constants.Limelight.Offsets.FL_ROLL,
            Constants.Limelight.Offsets.FL_PITCH,
            Constants.Limelight.Offsets.FL_YAW
        );

        LimelightHelpers.setCameraPose_RobotSpace(
            "limelight-fr",
            Constants.Limelight.Offsets.FR_X_OFFSET,
            Constants.Limelight.Offsets.FR_Z_OFFSET,
            Constants.Limelight.Offsets.FR_Y_OFFSET,
            Constants.Limelight.Offsets.FR_ROLL,
            Constants.Limelight.Offsets.FR_PITCH,
            Constants.Limelight.Offsets.FR_YAW
        );

        LimelightHelpers.setCameraPose_RobotSpace(
            "limelight-bl",
            Constants.Limelight.Offsets.BL_X_OFFSET,
            Constants.Limelight.Offsets.BL_Z_OFFSET,
            Constants.Limelight.Offsets.BL_Y_OFFSET,
            Constants.Limelight.Offsets.BL_ROLL,
            Constants.Limelight.Offsets.BL_PITCH,
            Constants.Limelight.Offsets.BL_YAW
        );

        LimelightHelpers.setCameraPose_RobotSpace(
            "limelight-br",
            Constants.Limelight.Offsets.BR_Z_OFFSET,
            Constants.Limelight.Offsets.BR_X_OFFSET,
            Constants.Limelight.Offsets.BR_Y_OFFSET,
            Constants.Limelight.Offsets.BR_ROLL,
            Constants.Limelight.Offsets.BR_PITCH,
            Constants.Limelight.Offsets.BR_YAW
        );
    }

    /**
     * Gets the Limelight instance
     * 
     * @return The {@link Limelight} instance
     */
    public static Limelight system() {
        if (instance == null) {
            instance = new Limelight();
        }

        return instance;
    }
}