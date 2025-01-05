package raidzero.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import raidzero.robot.subsystems.TelescopingArm;

public class MoveTelescopingArm extends Command {
    private TelescopingArm arm;
    private double x, y;

    private double[] setpoints;

    public MoveTelescopingArm(double x, double y, double wristAngle) {
        this.arm = TelescopingArm.system();
        this.x = x;
        this.y = y;

        setpoints = null;

        addRequirements(this.arm);
    }

    @Override
    public void execute() {
        setpoints = arm.moveArmTo(x, y);
    }

    @Override
    public boolean isFinished() {
        // within 2 rotations of target (change this)
        return ((arm.getTelescopePosition() > setpoints[0] - 2) && (arm.getTelescopePosition() < setpoints[0] + 2))
                && (arm.getArmPosition() > setpoints[1] - 2) && (arm.getArmPosition() < setpoints[1] + 2);
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopAll();
    }
}
