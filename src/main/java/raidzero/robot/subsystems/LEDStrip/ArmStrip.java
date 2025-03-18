
package raidzero.robot.subsystems.LEDStrip;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import raidzero.robot.Constants;
import raidzero.robot.subsystems.climb.ClimbJoint;
import raidzero.robot.subsystems.drivetrain.Swerve;
import raidzero.robot.subsystems.telescopingarm.Arm;
import raidzero.robot.subsystems.telescopingarm.CoralIntake;

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
        updateStates();

        if (DriverStation.isEStopped()) {
            loopEstopped();
        } else if (DriverStation.isDisabled()) {
            loopDisabled();
        } else if (DriverStation.isAutonomousEnabled()) {
            loopAutonomous();
        } else if (DriverStation.isAutonomous() && !DriverStation.isAutonomousEnabled()) {
            candle.clearAnimation(0);
            candle.clearAnimation(1);
        } else if (DriverStation.isTeleop() && !DriverStation.isTeleopEnabled()) {
            candle.clearAnimation(0);
            candle.clearAnimation(1);
        } else if (DriverStation.isTeleopEnabled()) {
            loopTeleop();
        } else if (DriverStation.isTestEnabled()) {
            loopTest();
        }
    }

    /**
     * Updates the state variables for the CANdle LED strip
     */
    private void updateStates() {
        armIsLegal = arm.getJointPosition() >= Constants.CANdle.ARM_JOINT_LOWER_BOUND &&
            arm.getJointPosition() <= Constants.CANdle.ARM_JOINT_UPPER_BOUND;

        coralTooDown = CoralIntake.system().getTopLaserDistance() > Constants.TelescopingArm.Intake.TOP_LASER_THRESHOLD_MM &&
            CoralIntake.system().getBottomLaserDistance() < Constants.TelescopingArm.Intake.TOP_LASER_THRESHOLD_MM;

        coralTooUp = CoralIntake.system().getTopLaserDistance() < Constants.TelescopingArm.Intake.TOP_LASER_THRESHOLD_MM &&
            CoralIntake.system().getBottomLaserDistance() > Constants.TelescopingArm.Intake.TOP_LASER_THRESHOLD_MM;

        coralIsIn = CoralIntake.system().getTopLaserDistance() < Constants.TelescopingArm.Intake.TOP_LASER_THRESHOLD_MM &&
            CoralIntake.system().getBottomLaserDistance() < Constants.TelescopingArm.Intake.TOP_LASER_THRESHOLD_MM;
    }

    /**
     * The E-stopped loop of the CANdle LED strip
     */
    private void loopEstopped() {
        if (animation2Applied || animation3Applied) {
            resetAnimation();
        }

        if (!animationApplied) {
            candle.clearAnimation(0);
            candle.clearAnimation(1);
            candle.animate(new StrobeAnimation(255, 0, 0, 0, 0.001, -1));
            animationApplied = true;
            animation2Applied = false;
            animation3Applied = false;
        }
    }

    /**
     * The disabled loop of the CANdle LED strip
     */
    private void loopDisabled() {
        if (DriverStation.isAutonomous() && DriverStation.getMatchTime() > 0.0) {
            if (animation2Applied || animation3Applied) {
                resetAnimation();
            }

            if (!animationApplied) {
                candle.clearAnimation(0);
                candle.clearAnimation(1);
                candle.animate(new StrobeAnimation(255, 0, 0, 0, 0.001, -1));
                animationApplied = true;
                animation2Applied = false;
                animation3Applied = false;
            }
        } else if (ClimbJoint.system().isDeployed().getAsBoolean()) {
            if (!animation3Applied) {
                candle.clearAnimation(0);
                candle.clearAnimation(1);
                candle.animate(new RainbowAnimation(255, 1.0, -1));
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
                    candle.setLEDs(255, 0, 0, 0, 0, 4);
                    candle.setLEDs(0, 0, 0, 0, 4, 4);
                    candle.setLEDs(255, 0, 0, 0, 8, 25);
                    candle.setLEDs(0, 0, 0, 0, 33, 27);
                } else {
                    candle.setLEDs(0, 0, 0, 0, 0, 4);
                    candle.setLEDs(255, 0, 0, 0, 4, 4);
                    candle.setLEDs(0, 0, 0, 0, 8, 25);
                    candle.setLEDs(255, 0, 0, 0, 33, 27);
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
    }

    /**
     * The autonomous loop of the CANdle LED strip
     */
    private void loopAutonomous() {
        if (animationApplied || animation2Applied || animation3Applied) {
            resetAnimation();
        }

        if (strobeTimer.hasElapsed(strobeInterval)) {
            if (strobeAlternate) {
                candle.setLEDs(255, 165, 0, 0, 0, 4);
                candle.setLEDs(0, 255, 0, 0, 4, 4);
                candle.setLEDs(255, 165, 0, 0, 8, 25);
                candle.setLEDs(0, 255, 0, 0, 33, 27);
            } else {
                candle.setLEDs(0, 255, 0, 0, 0, 4);
                candle.setLEDs(255, 165, 0, 0, 4, 4);
                candle.setLEDs(0, 255, 0, 0, 8, 25);
                candle.setLEDs(255, 165, 0, 0, 33, 27);
            }

            strobeAlternate = !strobeAlternate;
            strobeTimer.reset();
        }
    }

    /**
     * The teleop loop of the CANdle LED strip
     */
    private void loopTeleop() {
        if (ClimbJoint.system().isDeployed().getAsBoolean()) {
            candle.animate(new StrobeAnimation(0, 0, 255, 0, 0.05, -1));
        } else if (coralTooDown) {
            if (!animation2Applied) {
                candle.clearAnimation(0);
                candle.clearAnimation(1);
                candle.animate(new ColorFlowAnimation(250, 160, 10, 0, 0.75, 25, Direction.Backward, 8), 0);
                candle.animate(new ColorFlowAnimation(250, 160, 10, 0, 0.75, 27, Direction.Forward, 33), 1);
                animationApplied = false;
                animation2Applied = true;
                animation3Applied = false;
            }
        } else if (coralTooUp) {
            if (!animation2Applied) {
                candle.clearAnimation(0);
                candle.clearAnimation(1);
                candle.animate(new ColorFlowAnimation(250, 160, 10, 0, 0.75, 25, Direction.Forward, 8), 0);
                candle.animate(new ColorFlowAnimation(250, 160, 10, 0, 0.75, 27, Direction.Backward, 33), 1);
                animationApplied = false;
                animation2Applied = true;
                animation3Applied = false;
            }
        } else if (coralIsIn && !Swerve.system().isArmDeployable().getAsBoolean()) {
            if (animationApplied || animation2Applied || animation3Applied) {
                resetAnimation();
            }

            if (strobeTimer.hasElapsed(strobeInterval)) {
                if (strobeAlternate) {
                    candle.setLEDs(0, 255, 0, 0, 0, 4);
                    candle.setLEDs(255, 0, 0, 0, 4, 4);
                    candle.setLEDs(0, 255, 0, 0, 8, 25);
                    candle.setLEDs(255, 0, 0, 0, 33, 27);
                } else {
                    candle.setLEDs(255, 0, 0, 0, 0, 4);
                    candle.setLEDs(0, 255, 0, 0, 4, 4);
                    candle.setLEDs(255, 0, 0, 0, 8, 25);
                    candle.setLEDs(0, 255, 0, 0, 33, 27);
                }

                strobeAlternate = !strobeAlternate;
                strobeTimer.reset();
            }
        } else if (!Swerve.system().isArmDeployable().getAsBoolean()) {
            if (animationApplied || animation2Applied || animation3Applied) {
                resetAnimation();
            }

            if (strobeTimer.hasElapsed(strobeInterval)) {
                if (strobeAlternate) {
                    candle.setLEDs(255, 10, 250, 0, 0, 4);
                    candle.setLEDs(255, 0, 0, 0, 4, 4);
                    candle.setLEDs(255, 10, 250, 0, 8, 25);
                    candle.setLEDs(255, 0, 0, 0, 33, 27);
                } else {
                    candle.setLEDs(255, 0, 0, 0, 0, 4);
                    candle.setLEDs(255, 10, 250, 0, 4, 4);
                    candle.setLEDs(255, 0, 0, 0, 8, 25);
                    candle.setLEDs(255, 10, 250, 0, 33, 27);
                }

                strobeAlternate = !strobeAlternate;
                strobeTimer.reset();
            }
        } else if (coralIsIn && Swerve.system().isArmDeployable().getAsBoolean()) {
            if (animationApplied || animation2Applied || animation3Applied) {
                resetAnimation();
            }

            if (strobeTimer.hasElapsed(strobeInterval)) {
                if (strobeAlternate) {
                    candle.setLEDs(255, 10, 250, 0, 0, 4);
                    candle.setLEDs(0, 255, 0, 0, 4, 4);
                    candle.setLEDs(255, 10, 250, 0, 8, 25);
                    candle.setLEDs(0, 255, 0, 0, 33, 27);
                } else {
                    candle.setLEDs(0, 255, 0, 0, 0, 4);
                    candle.setLEDs(255, 10, 250, 0, 4, 4);
                    candle.setLEDs(0, 255, 0, 0, 8, 25);
                    candle.setLEDs(255, 10, 250, 0, 33, 27);
                }

                strobeAlternate = !strobeAlternate;
                strobeTimer.reset();
            }
        } else if (Swerve.system().isArmDeployable().getAsBoolean()) {
            if (animationApplied || animation2Applied || animation3Applied) {
                resetAnimation();
            }

            if (strobeTimer.hasElapsed(strobeInterval)) {
                if (strobeAlternate) {
                    candle.setLEDs(255, 10, 250, 0, 0, 4);
                    candle.setLEDs(0, 0, 0, 0, 4, 4);
                    candle.setLEDs(255, 10, 250, 0, 8, 25);
                    candle.setLEDs(0, 0, 0, 0, 33, 27);
                } else {
                    candle.setLEDs(0, 0, 0, 0, 0, 4);
                    candle.setLEDs(255, 10, 250, 0, 4, 4);
                    candle.setLEDs(0, 0, 0, 0, 8, 25);
                    candle.setLEDs(255, 10, 250, 0, 33, 27);
                }

                strobeAlternate = !strobeAlternate;
                strobeTimer.reset();
            }
        }
    }

    /**
     * The test loop of the CANdle LED strip
     */
    private void loopTest() {
        if (!animationApplied) {
            candle.animate(new StrobeAnimation(255, 165, 0, 0, 0.05, -1));
            animationApplied = true;
        }
    }

    /**
     * Clears CAndle animaitons
     */
    public void resetAnimation() {
        candle.clearAnimation(0);
        candle.clearAnimation(1);
        animationApplied = false;
        animation2Applied = false;
        animation3Applied = false;
    }

    /**
     * Plays the match end animation
     */
    public void matchEndAnimation() {
        candle.clearAnimation(0);
        candle.clearAnimation(1);
        candle.animate(new RainbowAnimation(255, 0.75, -1));
    }

    /**
     * Sets the LED to red, green, blue in alternating sequence
     */
    private void testCandleSequence() {
        candle.setLEDs(255, 0, 0, 0, 0, 5);
        candle.setLEDs(0, 255, 0, 0, 5, 5);
        candle.setLEDs(0, 0, 255, 0, 10, 5);
        candle.setLEDs(255, 0, 0, 0, 15, 5);
        candle.setLEDs(0, 255, 0, 0, 20, 5);
        candle.setLEDs(0, 0, 255, 0, 25, 5);
        candle.setLEDs(255, 0, 0, 0, 30, 5);
        candle.setLEDs(0, 255, 0, 0, 35, 5);
        candle.setLEDs(0, 0, 255, 0, 40, 5);
        candle.setLEDs(255, 0, 0, 0, 45, 5);
        candle.setLEDs(0, 255, 0, 0, 50, 5);
        candle.setLEDs(0, 0, 255, 0, 55, 5);
        candle.setLEDs(255, 0, 0, 0, 60, 5);
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
