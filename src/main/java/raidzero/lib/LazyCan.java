package raidzero.lib;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import edu.wpi.first.wpilibj.DriverStation;

public class LazyCan {
    private LaserCan laserCan;
    private int canId;

    private Measurement measurement;

    private double threshold;

    /**
     * Creates a new LaserCAN sensor.
     *
     * @param canId The CAN ID for the LaserCAN sensor
     */
    public LazyCan(int canId) {
        laserCan = new LaserCan(canId);
        this.canId = canId;
    }

    /**
     * Gets the distance in mm from the sensor
     *
     * @return The distance in mm, -1 if the sensor cannot be found
     */
    public int getDistanceMm() {
        measurement = laserCan.getMeasurement();

        return measurement != null ? measurement.distance_mm : -1;
    }

    /**
     * Checks if the LaserCan finds an object within the distance threshold
     *
     * @return True if there is an object within the distance threshold, false otherwise
     */
    public boolean withinThreshold() {
        return getDistanceMm() <= threshold;
    }

    /**
     * Sets the reigon of interest for the lasercan
     *
     * @param x the x start position for the reigon
     * @param y the y start position for the reigon
     * @param w the width of the reigon
     * @param h the height of the reigon
     * @return The current {@link LazyCan} instance
     */
    public LazyCan withRegionOfInterest(int x, int y, int w, int h) {
        try {
            laserCan.setRegionOfInterest(new RegionOfInterest(x, y, w, h));
        } catch (ConfigurationFailedException e) {
            DriverStation.reportError("LaserCan " + canId + ": RegionOfInterest Configuration failed! " + e, true);
        }

        return this;
    }

    /**
     * Sets the reigon of interest for the lasercan
     * 
     * @param regionOfInterest The region of interest
     * @return The current {@link LazyCan} instance
     */
    public LazyCan withRegionOfInterest(RegionOfInterest regionOfInterest) {
        try {
            laserCan.setRegionOfInterest(regionOfInterest);
        } catch (ConfigurationFailedException e) {
            DriverStation.reportError("LaserCan " + canId + ": RegionOfInterest Configuration failed! " + e, true);
        }

        return this;
    }

    /**
     * Sets the ranging mode of the LaserCan
     *
     * @param rangingMode The ranging mode
     * @return The current {@link LazyCan} instance
     */
    public LazyCan withRangingMode(RangingMode rangingMode) {
        try {
            laserCan.setRangingMode(rangingMode);
        } catch (ConfigurationFailedException e) {
            System.out.println("LaserCan " + canId + ": RangingMode Configuration failed! " + e);
        }

        return this;
    }

    /**
     * Sets the timing budget of the LaserCan
     *
     * @param timingBudget The timing budget
     * @return The current {@link LazyCan} instance
     */
    public LazyCan withTimingBudget(TimingBudget timingBudget) {
        try {
            laserCan.setTimingBudget(timingBudget);
        } catch (ConfigurationFailedException e) {
            DriverStation.reportError("LaserCan " + canId + ": TimingBudget Configuration failed! " + e, true);
        }

        return this;
    }

    /**
     * Sets the distance threshold of the LaserCan
     *
     * @param threshold The threshold in milimeters
     * @return The current {@link LazyCan} instance
     */
    public LazyCan withThreshold(double threshold) {
        this.threshold = threshold;
        return this;
    }
}