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

    private RangingMode rangingMode;
    private RegionOfInterest regionOfInterest;
    private TimingBudget timingBudget;

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

    public boolean withinThreshold() {
        measurement = laserCan.getMeasurement();
        
        return measurement != null ? measurement.distance_mm <= threshold : false;
    }

    public LazyCan withRegionOfInterest(int x, int y, int w, int h) {
        regionOfInterest = new RegionOfInterest(x, y, w, h);

        try {
            laserCan.setRegionOfInterest(regionOfInterest);
        } catch (ConfigurationFailedException e) {
            DriverStation.reportError("LaserCan " + canId + ": RegionOfInterest Configuration failed! " + e, true);
        }

        return this;
    }

    public void build() {
        if (regionOfInterest == null) {
            regionOfInterest = new RegionOfInterest(4, 4, 8, 8);
        }
        if (rangingMode == null) {
            rangingMode = RangingMode.SHORT;
        }
        if (timingBudget == null) {
            timingBudget = TimingBudget.TIMING_BUDGET_33MS;
        }
        if (threshold == 0) {
            threshold = 500;
        }

        try {
            laserCan.setRegionOfInterest(regionOfInterest);
            laserCan.setRangingMode(rangingMode);
            laserCan.setTimingBudget(timingBudget);
        } catch (ConfigurationFailedException e) {
            DriverStation.reportError("LaserCan " + canId + ": Configuration failed! " + e, true);
        }
    }

    public LazyCan withRangingMode(RangingMode rangingMode) {
        this.rangingMode = rangingMode;
        try {
            laserCan.setRangingMode(rangingMode);
        } catch (ConfigurationFailedException e) {
            System.out.println("LaserCan " + canId + ": RangingMode Configuration failed! " + e);
        }
        return this;
    }

    public LazyCan withTimingBudget(TimingBudget timingBudget) {
        this.timingBudget = timingBudget;
        try {
            laserCan.setTimingBudget(timingBudget);
        } catch (ConfigurationFailedException e) {
            DriverStation.reportError("LaserCan " + canId + ": TimingBudget Configuration failed! " + e, true);
        }
        return this;
    }

    public LazyCan withThreshold(double threshold) {
        this.threshold = threshold;
        return this;
    }
}