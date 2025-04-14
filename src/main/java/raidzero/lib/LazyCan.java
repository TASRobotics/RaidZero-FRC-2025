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

    private RegionOfInterest regionOfInterest;

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
        measurement = laserCan.getMeasurement();

        return measurement != null ? measurement.distance_mm : -1;
    }

    /**
     * Returns true if the LaserCan finds an object within the distance threshold
     *
     * @return if there is an object within the distance threshold
     */
    public boolean withinThreshold() {
        measurement = laserCan.getMeasurement();

        return measurement != null ? measurement.distance_mm <= threshold : false;
    }

    /**
     * Sets the reigon of interest for the lasercan
     *
     * @param x the x start position for the reigon
     * @param y the y start position for the reigon
     * @param w the width of the reigon
     * @param h the height of the reigon
     * @return the current LazyCan Object
     */
    public LazyCan withRegionOfInterest(int x, int y, int w, int h) {
        regionOfInterest = new RegionOfInterest(x, y, w, h);

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
     * @param rangingMode the new ranging mode
     * @return the current LazyCan Object
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
     * @param timingBudget the new timing budget
     * @return the current LazyCan Object
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
     * @param threshold the new threshold in milimeters
     * @return the current LazyCan object
     */
    public LazyCan withThreshold(double threshold) {
        this.threshold = threshold;
        return this;
    }
}