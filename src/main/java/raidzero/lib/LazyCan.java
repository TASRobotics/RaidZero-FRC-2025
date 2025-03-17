package raidzero.lib;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.DriverStation;

public class LazyCan extends LaserCan {
    private int canId;

    private RangingMode rangingMode;
    private RegionOfInterest regionOfInterest;
    private TimingBudget timingBudget;

    private Measurement measurement;

    private int threshold;

    /**
     * Creates a new LaserCAN sensor.
     *
     * @param canId The CAN ID for the LaserCAN sensor
     */
    public LazyCan(int canId) {
        super(canId);
        this.canId = canId;
    }

    /**
     * Gets the distance in mm from the sensor
     *
     * @return The distance in mm, -1 if the sensor cannot be found
     */
    public int getDistanceMm() {
        measurement = getMeasurement();

        return measurement != null ? measurement.distance_mm : -1;
    }

    public boolean getStatus() {
        measurement = getMeasurement();
        
        return measurement != null ? getMeasurement().distance_mm <= threshold : false;
    }

    public LazyCan withRegionOfInterest(int x, int y, int w, int h) {
        regionOfInterest = new RegionOfInterest(x, y, w, h);

        try {
            this.setRegionOfInterest(regionOfInterest);
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
            this.setRegionOfInterest(regionOfInterest);
            this.setRangingMode(rangingMode);
            this.setTimingBudget(timingBudget);
        } catch (ConfigurationFailedException e) {
            DriverStation.reportError("LaserCan " + canId + ": Configuration failed! " + e, true);
        }
    }

    public LazyCan withRangingMode(RangingMode rangingMode) {
        this.rangingMode = rangingMode;
        try {
            this.setRangingMode(rangingMode);
        } catch (ConfigurationFailedException e) {
            System.out.println("LaserCan " + canId + ": RangingMode Configuration failed! " + e);
        }
        return this;
    }

    public LazyCan withTimingBudget(TimingBudget timingBudget) {
        this.timingBudget = timingBudget;
        try {
            this.setTimingBudget(timingBudget);
        } catch (ConfigurationFailedException e) {
            DriverStation.reportError("LaserCan " + canId + ": TimingBudget Configuration failed! " + e, true);
        }
        return this;
    }

    public LazyCan withThreshold(int threshold) {
        this.threshold = threshold;
        return this;
    }

    @Override
    public void close() throws Exception {
        super.close();
    }
}