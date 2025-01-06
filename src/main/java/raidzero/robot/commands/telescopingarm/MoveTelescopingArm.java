package raidzero.robot.commands.telescopingarm;

import edu.wpi.first.wpilibj2.command.Command;
import raidzero.robot.Constants;
import raidzero.robot.subsystems.TelescopingArm;

public class MoveTelescopingArm extends Command {
    private TelescopingArm arm;
    private double armJointSetpoint, telescopeSetpoint;

    public MoveTelescopingArm(double x, double y, double wristAngle) {
        this.arm = TelescopingArm.system();

        this.telescopeSetpoint = calculateTelescopeHeight(x, y);
        this.armJointSetpoint = calculateArmAngle(x, y);

        addRequirements(this.arm);
    }

    @Override
    public void execute() {
        arm.moveTo(telescopeSetpoint, armJointSetpoint);
    }

    @Override
    public boolean isFinished() {
        // within 2 rotations of target (change this)
        return ((arm.getTelescopePosition() > telescopeSetpoint - Constants.TelescopingArm.Telescope.POSITION_TOLERANCE_ROTATIONS)
                && (arm.getTelescopePosition() < telescopeSetpoint + Constants.TelescopingArm.Telescope.POSITION_TOLERANCE_ROTATIONS))
                && (arm.getArmPosition() > armJointSetpoint - Constants.TelescopingArm.ArmJoint.POSITION_TOLERANCE_ROTATIONS)
                && (arm.getArmPosition() < armJointSetpoint + Constants.TelescopingArm.ArmJoint.POSITION_TOLERANCE_ROTATIONS);
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopAll();
    }

    /**
     * Calculates the target telescope position given the x and y setpoints
     * 
     * @param x the x setpoint in meters
     * @param y the y setpoint in meters
     * @return target motor position in rotations
     * @throws IllegalStateException if the calculated target height is higher than the maximum height
     */
    private double calculateTelescopeHeight(double x, double y) throws IllegalStateException {
        double height = Math.sqrt(x * x + y * y);

        if (height > Constants.TelescopingArm.Telescope.MAX_LENGTH_M)
            throw new IllegalStateException("Arm setpoint too high");

        return height * Constants.TelescopingArm.Telescope.CONVERSION_FACTOR;
    }

    /**
     * Calculates the target arm position given the x and y setpoints
     * 
     * @param x the x setpoint in meters
     * @param y the y setpoint in meters
     * @return the target arm position in rotations
     */
    private double calculateArmAngle(double x, double y) {
        return Math.atan2(y, x) * Constants.TelescopingArm.ArmJoint.CONVERSION_FACTOR;
    }
}