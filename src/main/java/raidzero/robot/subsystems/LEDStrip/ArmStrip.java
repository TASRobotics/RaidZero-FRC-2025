
package raidzero.robot.subsystems.LEDStrip;

import raidzero.robot.Constants;
import raidzero.robot.subsystems.climb.ClimbJoint;
import raidzero.robot.subsystems.drivetrain.Swerve;
import raidzero.robot.subsystems.telescopingarm.Arm;
import raidzero.robot.subsystems.telescopingarm.CoralIntake;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ArmStrip implements Subsystem {

    private CANdle candle;
    private Arm arm;

    private boolean armIsLegal, coralTooDown, coralTooUp, coralIsIn = false;

    private boolean strobeAlternate = false;
    private Timer strobeTimer = new Timer();
    private double strobeInterval = 0.25;

    private boolean animationApplied, animation2Applied, animation3Applied = false;

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
        strobeTimer.start();
    }

    /**
     * The main loop of the CANdle LED strip
     */
    public void loop() {
        armIsLegal = arm.getJointPosition() >= Constants.CANdle.ARM_JOINT_LOWER_BOUND &&
            arm.getJointPosition() <= Constants.CANdle.ARM_JOINT_UPPER_BOUND;

        coralTooDown = CoralIntake.system().getTopLaserDistance() > Constants.TelescopingArm.Intake.LASERCAN_DISTANCE_THRESHOLD_MM &&
            CoralIntake.system().getBottomLaserDistance() < Constants.TelescopingArm.Intake.LASERCAN_DISTANCE_THRESHOLD_MM;

        coralTooUp = CoralIntake.system().getTopLaserDistance() < Constants.TelescopingArm.Intake.LASERCAN_DISTANCE_THRESHOLD_MM &&
            CoralIntake.system().getBottomLaserDistance() > Constants.TelescopingArm.Intake.LASERCAN_DISTANCE_THRESHOLD_MM;

        coralIsIn = CoralIntake.system().getTopLaserDistance() < Constants.TelescopingArm.Intake.LASERCAN_DISTANCE_THRESHOLD_MM &&
            CoralIntake.system().getBottomLaserDistance() < Constants.TelescopingArm.Intake.LASERCAN_DISTANCE_THRESHOLD_MM;

        if (DriverStation.isDisabled()) {
            if (ClimbJoint.system().isDeployed().getAsBoolean()) {
                if (!animation3Applied) {
                    candle.clearAnimation(0);
                    candle.clearAnimation(1);
                    candle.animate(new RainbowAnimation(255, 0.75, -1));
                    animationApplied = false;
                    animation2Applied = false;
                    animation3Applied = true;
                }
            } else if (armIsLegal && ClimbJoint.system().getPosition() < 0.1 && !ClimbJoint.system().isDeployed().getAsBoolean()) {
                if (!animation2Applied) {
                    candle.clearAnimation(0);
                    candle.clearAnimation(1);
                    candle.animate(new StrobeAnimation(0, 255, 0, 0, 0.001, -1));
                    animation2Applied = true;
                    animationApplied = false;
                    animation3Applied = false;
                }
            } else if (!armIsLegal && ClimbJoint.system().getPosition() < 0.1 && !ClimbJoint.system().isDeployed().getAsBoolean()) {
                if (animationApplied || animation2Applied || animation3Applied) {
                    candle.clearAnimation(0);
                    candle.clearAnimation(1);
                    animationApplied = false;
                    animation2Applied = false;
                    animation3Applied = false;
                }

                if (strobeTimer.hasElapsed(strobeInterval)) {
                    if (strobeAlternate) {
                        candle.setLEDs(255, 0, 0, 0, 8, 25);
                        candle.setLEDs(0, 0, 0, 0, 33, 25);
                    } else {
                        candle.setLEDs(0, 0, 0, 0, 8, 25);
                        candle.setLEDs(255, 0, 0, 0, 33, 25);
                    }

                    strobeAlternate = !strobeAlternate;
                    strobeTimer.reset();
                }
            } else if (!armIsLegal) {
                candle.setLEDs(255, 0, 0);
                candle.clearAnimation(0);
                animationApplied = false;
                animation2Applied = false;
                animation3Applied = false;
            } else if (armIsLegal) {
                candle.setLEDs(0, 255, 0);
                candle.clearAnimation(0);
                animationApplied = false;
                animation2Applied = false;
                animation3Applied = false;
            }
        } else if (DriverStation.isAutonomousEnabled()) {
            if (strobeTimer.hasElapsed(strobeInterval)) {
                if (strobeAlternate) {
                    candle.setLEDs(255, 165, 0, 0, 0, 33);
                    candle.setLEDs(0, 255, 0, 0, 38, 33);
                } else {
                    candle.setLEDs(0, 255, 0, 0, 0, 33);
                    candle.setLEDs(255, 165, 0, 0, 33, 38);
                }

                strobeAlternate = !strobeAlternate;
                strobeTimer.reset();
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
            } else if (coralTooDown) {
                if (!animation2Applied) {
                    candle.clearAnimation(0);
                    candle.clearAnimation(1);
                    candle.animate(new ColorFlowAnimation(250, 160, 10, 0, 0.75, 25, Direction.Backward, 8), 0);
                    candle.animate(new ColorFlowAnimation(250, 160, 10, 0, 0.75, 25, Direction.Forward, 33), 1);
                    animationApplied = false;
                    animation2Applied = true;
                    animation3Applied = false;
                }
            } else if (coralTooUp) {
                if (!animation2Applied) {
                    candle.clearAnimation(0);
                    candle.clearAnimation(1);
                    candle.animate(new ColorFlowAnimation(250, 160, 10, 0, 0.75, 25, Direction.Forward, 8), 0);
                    candle.animate(new ColorFlowAnimation(250, 160, 10, 0, 0.75, 25, Direction.Backward, 33), 1);
                    animationApplied = false;
                    animation2Applied = true;
                    animation3Applied = false;
                }
            } else if (coralIsIn && !Swerve.system().isArmDeployable().getAsBoolean()) {
                if (animationApplied || animation2Applied || animation3Applied) {
                    candle.clearAnimation(0);
                    candle.clearAnimation(1);
                    animationApplied = false;
                    animation2Applied = false;
                    animation3Applied = false;
                }

                if (strobeTimer.hasElapsed(strobeInterval)) {
                    if (strobeAlternate) {
                        candle.setLEDs(0, 255, 0, 0, 8, 25);
                        candle.setLEDs(255, 0, 0, 0, 33, 25);
                    } else {
                        candle.setLEDs(255, 0, 0, 0, 8, 25);
                        candle.setLEDs(0, 255, 0, 0, 33, 25);
                    }

                    strobeAlternate = !strobeAlternate;
                    strobeTimer.reset();
                }
            } else if (!Swerve.system().isArmDeployable().getAsBoolean()) {
                if (animationApplied || animation2Applied || animation3Applied) {
                    candle.clearAnimation(0);
                    candle.clearAnimation(1);
                    animationApplied = false;
                    animation2Applied = false;
                    animation3Applied = false;
                }

                if (strobeTimer.hasElapsed(strobeInterval)) {
                    if (strobeAlternate) {
                        candle.setLEDs(255, 10, 250, 0, 8, 25);
                        candle.setLEDs(255, 0, 0, 0, 33, 25);
                    } else {
                        candle.setLEDs(255, 0, 0, 0, 8, 25);
                        candle.setLEDs(255, 10, 250, 0, 33, 25);
                    }

                    strobeAlternate = !strobeAlternate;
                    strobeTimer.reset();
                }
            } else if (coralIsIn && Swerve.system().isArmDeployable().getAsBoolean()) {
                if (animationApplied || animation2Applied || animation3Applied) {
                    candle.clearAnimation(0);
                    candle.clearAnimation(1);
                    animationApplied = false;
                    animation2Applied = false;
                    animation3Applied = false;
                }

                if (strobeTimer.hasElapsed(strobeInterval)) {
                    if (strobeAlternate) {
                        candle.setLEDs(255, 10, 250, 0, 8, 25);
                        candle.setLEDs(0, 255, 0, 0, 33, 25);
                    } else {
                        candle.setLEDs(0, 255, 0, 0, 8, 25);
                        candle.setLEDs(255, 10, 250, 0, 33, 25);
                    }

                    strobeAlternate = !strobeAlternate;
                    strobeTimer.reset();
                }
            } else if (Swerve.system().isArmDeployable().getAsBoolean()) {
                if (animationApplied || animation2Applied || animation3Applied) {
                    candle.clearAnimation(0);
                    candle.clearAnimation(1);
                    animationApplied = false;
                    animation2Applied = false;
                    animation3Applied = false;
                }

                if (strobeTimer.hasElapsed(strobeInterval)) {
                    if (strobeAlternate) {
                        candle.setLEDs(255, 10, 250, 0, 8, 25);
                        candle.setLEDs(0, 0, 0, 0, 33, 25);
                    } else {
                        candle.setLEDs(0, 0, 0, 0, 8, 25);
                        candle.setLEDs(255, 10, 250, 0, 33, 25);
                    }

                    strobeAlternate = !strobeAlternate;
                    strobeTimer.reset();
                }
            }
        } else if (DriverStation.isTestEnabled()) {
            if (!animationApplied) {
                candle.animate(new StrobeAnimation(255, 165, 0, 0, 0.05, -1));
                animationApplied = true;
            }
        }
    }

    /**
     * Clears CAndle animaitons
     */
    public void resetAnimation() {
        animationApplied = false;
        animation2Applied = false;
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
