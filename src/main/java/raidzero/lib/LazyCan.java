package raidzero.lib;

import au.grapplerobotics.LaserCan;

public class LazyCan extends LaserCan {

    /**
     * Creates a new LaserCAN sensor.
     * 
     * @param canId The CAN ID for the LaserCAN sensor
     */
    public LazyCan(int canId) {
        super(canId);
    }

    /**
     * Gets the distance in mm from the sensor
     * 
     * @return The distance in mm, -1 if the sensor cannot be found
     */
    public int getDistanceMm() {
        Measurement measurement = getMeasurement();

        return measurement != null ? measurement.distance_mm : -1;
    }
}