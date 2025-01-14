package raidzero.robot.subsystems;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidzero.robot.Constants;
import raidzero.robot.TunerConstants;
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

    private boolean ignoreFrontLime = false;
    private boolean ignoreLeftLime = false;
    private boolean ignoreRightLime = false;
    private boolean ignoreBackLime = false;
    private boolean ignoreAllLimes = false;

    private LimelightHelpers.PoseEstimate limeFront, limeLeft, limeRight, limeBack;
    private LimelightHelpers.PoseEstimate limeFrontPrev, limeLeftPrev, limeRightPrev, limeBackPrev;

    private Swerve swerve = Swerve.system();
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

    @Override
    public void periodic() {
        if (swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble() > 720) {
            ignoreFrontLime = true;
            ignoreLeftLime = true;
            ignoreRightLime = true;
            ignoreBackLime = true;
        } else {
            ignoreFrontLime = false;
            ignoreLeftLime = false;
            ignoreRightLime = false;
            ignoreBackLime = false;
        }

        LimelightHelpers.SetRobotOrientation(
            "limelight-front",
            swerve.getState().Pose.getRotation().getDegrees(),
            swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble(),
            0,
            0,
            0,
            0
        );
        limeFront = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-front");

        if (limeFront != null && limeFront.pose != null) {
            ignoreFrontLime = !validPose(limeFront.pose) ||
                (Math.abs(LimelightHelpers.getBotPose3d_wpiBlue("limelight-front").getZ()) > 0.4) ||
                (LimelightHelpers.getTA("limelight-front") < 0.1) ||
                (limeFrontPrev != null && getLLposesDist(limeFront.pose, limeFrontPrev.pose) > 0.8) ||
                (limeFrontPrev != null && (getLLposesDist(limeFront.pose, limeFrontPrev.pose) /
                    (limeFront.timestampSeconds - limeFrontPrev.timestampSeconds)) > TunerConstants.kSpeedAt12Volts.baseUnitMagnitude()) ||
                (limeFront.rawFiducials.length > 0 && limeFront.rawFiducials[0].ambiguity > 0.5 &&
                    limeFront.rawFiducials[0].distToCamera > 3.5);

            if (!ignoreAllLimes && !ignoreFrontLime) {
                SmartDashboard.putBoolean("Fpose", true);

                swerve.addVisionMeasurement(
                    limeFront.pose,
                    Utils.fpgaToCurrentTime(limeFront.timestampSeconds),
                    VecBuilder.fill(0.7, 0.7, 9999999).div(LimelightHelpers.getTA("limelight-front"))
                );
            } else {
                SmartDashboard.putBoolean("Fpose", false);
            }

            limeFrontPrev = limeFront;
        }

        LimelightHelpers.SetRobotOrientation(
            "limelight-left",
            swerve.getState().Pose.getRotation().plus(Rotation2d.fromDegrees(Constants.Limelight.Offsets.LEFT_YAW)).getDegrees(),
            swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble(), Constants.Limelight.Offsets.LEFT_PITCH,
            0,
            0,
            0
        );
        limeLeft = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");

        if (limeLeft != null && limeLeft.pose != null) {
            ignoreLeftLime = limeLeft.tagCount == 0 ||
                !validPose(limeLeft.pose) ||
                (LimelightHelpers.getTA("limelight-left") < 0.1) ||
                (getLLposesDist(
                    limeLeft.pose, limeLeftPrev.pose
                ) > ((limeLeft.timestampSeconds - limeLeftPrev.timestampSeconds) * TunerConstants.kSpeedAt12Volts.baseUnitMagnitude())) ||
                (limeLeft.rawFiducials.length > 0 && limeLeft.rawFiducials[0].ambiguity > 0.5 &&
                    limeLeft.rawFiducials[0].distToCamera > 3.5);

            if (!ignoreAllLimes && !ignoreLeftLime) {
                SmartDashboard.putBoolean("Lpose", true);

                swerve.addVisionMeasurement(
                    new Pose2d(
                        limeLeft.pose.getX(),
                        limeLeft.pose.getY(),
                        swerve.getPigeon2().getRotation2d()
                    ),
                    limeLeft.timestampSeconds,
                    VecBuilder.fill(.1, .1, 9999999).div(LimelightHelpers.getTA("limelight-left"))
                );
            } else {
                SmartDashboard.putBoolean("Lpose", false);
            }

            limeLeftPrev = limeLeft;
        }

        LimelightHelpers.SetRobotOrientation(
            "limelight-right",
            swerve.getState().Pose.getRotation().minus(Rotation2d.fromDegrees(Constants.Limelight.Offsets.RIGHT_YAW)).getDegrees(),
            swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble(), Constants.Limelight.Offsets.RIGHT_PITCH,
            0,
            0,
            0
        );
        limeRight = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-right");

        if (limeRight != null && limeRight.pose != null) {
            ignoreRightLime = limeRight.tagCount == 0 ||
                !validPose(limeRight.pose) ||
                (LimelightHelpers.getTA("limelight-right") < 0.1) ||
                (getLLposesDist(
                    limeRight.pose, limeRightPrev.pose
                ) > ((limeRight.timestampSeconds - limeRightPrev.timestampSeconds) * TunerConstants.kSpeedAt12Volts.baseUnitMagnitude())) ||
                (limeRight.rawFiducials.length > 0 && limeRight.rawFiducials[0].ambiguity > 0.5 &&
                    limeRight.rawFiducials[0].distToCamera > 3.5);

            if (!ignoreAllLimes && !ignoreRightLime) {
                SmartDashboard.putBoolean("Rpose", true);

                swerve.addVisionMeasurement(
                    new Pose2d(
                        limeRight.pose.getX(),
                        limeRight.pose.getY(),
                        swerve.getPigeon2().getRotation2d()
                    ),
                    limeRight.timestampSeconds,
                    VecBuilder.fill(.1, .1, 9999999).div(LimelightHelpers.getTA("limelight-right"))
                );
            } else {
                SmartDashboard.putBoolean("Rpose", false);
            }

            limeRightPrev = limeRight;
        }

        LimelightHelpers.SetRobotOrientation(
            "limelight-back",
            swerve.getState().Pose.getRotation().plus(Rotation2d.fromDegrees(Constants.Limelight.Offsets.BACK_YAW)).getDegrees(),
            swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble(), Constants.Limelight.Offsets.BACK_PITCH,
            0,
            0,
            0
        );
        limeBack = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-back");

        if (limeBack != null && limeBack.pose != null) {
            ignoreBackLime = limeBack.tagCount == 0 ||
                !validPose(limeBack.pose) ||
                (LimelightHelpers.getTA("limelight-back") < 0.1) ||
                (getLLposesDist(
                    limeBack.pose, limeBackPrev.pose
                ) > ((limeBack.timestampSeconds - limeBackPrev.timestampSeconds) * TunerConstants.kSpeedAt12Volts.baseUnitMagnitude())) ||
                (limeBack.rawFiducials.length > 0 && limeBack.rawFiducials[0].ambiguity > 0.5 &&
                    limeBack.rawFiducials[0].distToCamera > 3.5);

            if (!ignoreAllLimes && !ignoreBackLime) {
                SmartDashboard.putBoolean("Bpose", true);

                swerve.addVisionMeasurement(
                    new Pose2d(
                        limeBack.pose.getX(),
                        limeBack.pose.getY(),
                        swerve.getPigeon2().getRotation2d()
                    ),
                    limeBack.timestampSeconds,
                    VecBuilder.fill(.1, .1, 9999999).div(LimelightHelpers.getTA("limelight-back"))
                );
            } else {
                SmartDashboard.putBoolean("Bpose", false);
            }

            limeBackPrev = limeBack;
        }
    }

    private boolean validPose(Pose2d pose) {
        return pose.getTranslation().getX() < 10 && pose.getTranslation().getY() < 10;
    }

    private double getLLposesDist(Pose2d pose1, Pose2d pose2) {
        return Math.sqrt(
            Math.pow(pose1.getTranslation().getX() - pose2.getTranslation().getX(), 2) +
                Math.pow(pose1.getTranslation().getY() - pose2.getTranslation().getY(), 2)
        );
    }

    public void initialize() {
        LimelightHelpers.setCameraPose_RobotSpace(
            "limelight-front",
            Constants.Limelight.Offsets.FRONT_X_OFFSET,
            Constants.Limelight.Offsets.FRONT_Z_OFFSET,
            Constants.Limelight.Offsets.FRONT_Y_OFFSET,
            Constants.Limelight.Offsets.FRONT_ROLL,
            Constants.Limelight.Offsets.FRONT_PITCH,
            Constants.Limelight.Offsets.FRONT_YAW
        );

        LimelightHelpers.setCameraPose_RobotSpace(
            "limelight-left",
            Constants.Limelight.Offsets.LEFT_X_OFFSET,
            Constants.Limelight.Offsets.LEFT_Z_OFFSET,
            Constants.Limelight.Offsets.LEFT_Y_OFFSET,
            Constants.Limelight.Offsets.LEFT_ROLL,
            Constants.Limelight.Offsets.LEFT_PITCH,
            Constants.Limelight.Offsets.LEFT_YAW
        );

        LimelightHelpers.setCameraPose_RobotSpace(
            "limelight-right",
            Constants.Limelight.Offsets.RIGHT_X_OFFSET,
            Constants.Limelight.Offsets.RIGHT_Z_OFFSET,
            Constants.Limelight.Offsets.RIGHT_Y_OFFSET,
            Constants.Limelight.Offsets.RIGHT_ROLL,
            Constants.Limelight.Offsets.RIGHT_PITCH,
            Constants.Limelight.Offsets.RIGHT_YAW
        );

        LimelightHelpers.setCameraPose_RobotSpace(
            "limelight-back",
            Constants.Limelight.Offsets.BACK_Z_OFFSET,
            Constants.Limelight.Offsets.BACK_X_OFFSET,
            Constants.Limelight.Offsets.BACK_Y_OFFSET,
            Constants.Limelight.Offsets.BACK_ROLL,
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