package raidzero.robot.lib.math;

/**
 * Class for calculating the velocities of a differential telescoping robot arm given a
 * path and a fixed speed.
 */

public class DifferentialTransformations {
    private final double x1, y1, x2, y2;
    private final double fixedSpeed;
    private final double accelRatio;

    public DifferentialTransformations(double x1, double y1, double x2, double y2, double fixedSpeed, double accelRatio) {
        this.x1 = x1;
        this.y1 = y1;
        this.x2 = x2;
        this.y2 = y2;
        this.fixedSpeed = fixedSpeed;
        this.accelRatio = accelRatio;
    }

    public double[] calculateVelocities(double x) {
        double y = y1 + (y2 - y1) * (x - x1) / (x2 - x1);
        double r = Math.sqrt(x * x + y * y);
        double theta = Math.atan2(y, x);
        double vr = fixedSpeed * Math.cos(theta);
        double vtheta = fixedSpeed * Math.sin(theta);
        return new double[]{vr, vtheta};
    }

    public double[] calculateDirection(double x, double y) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        double t = ((x - x1) * dx + (y - y1) * dy) / (dx * dx + dy * dy);
        double closestX = x1 + t * dx;
        double closestY = y1 + t * dy;
        double directionX = closestX - x;
        double directionY = closestY - y;
        double magnitude = Math.sqrt(directionX * directionX + directionY * directionY);
        return new double[]{fixedSpeed * directionX / magnitude, fixedSpeed * directionY / magnitude};
    }

    public double[] calculateCorrectedVelocities(double x, double y, double kP) {
        double[] velocities = calculateVelocities(x);
        double[] direction = calculateDirection(x, y);
        double correctedVr = velocities[0] + kP * direction[0];
        double correctedVtheta = velocities[1] + kP * direction[1];
        return new double[]{correctedVr, correctedVtheta};
    }

    public double[] calculateCorrectedAccelerations(double x, double y, double kP) {
        double[] correctedVelocities = calculateCorrectedVelocities(x, y, kP);
        double correctedAr = correctedVelocities[0] * accelRatio;
        double correctedAtheta = correctedVelocities[1] * accelRatio;
        return new double[]{correctedAr, correctedAtheta};
    }
    public void printCorrectedVelocitiesAndAccelerations(double x, double y, double kP) {
        double[] correctedVelocities = calculateCorrectedVelocities(x, y, kP);
        double[] correctedAccelerations = calculateCorrectedAccelerations(x, y, kP);
        
        System.out.println("Corrected Velocities: vr = " + correctedVelocities[0] + ", vtheta = " + correctedVelocities[1]);
        System.out.println("Corrected Accelerations: ar = " + correctedAccelerations[0] + ", atheta = " + correctedAccelerations[1]);
    }
}