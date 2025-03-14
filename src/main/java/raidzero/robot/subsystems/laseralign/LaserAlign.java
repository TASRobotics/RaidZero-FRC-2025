package raidzero.robot.subsystems.laseralign;

import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidzero.lib.LazyCan;

public class LaserAlign extends SubsystemBase{
    private LazyCan[] lc;
    private boolean[] statuses;
    private int[] distances;

    private static LaserAlign system;

    private LaserAlign(){
        lc = new LazyCan[2];
        statuses = new boolean[2];
        distances = new int[2];

        lc[0] = new LazyCan(2)
            .withRangingMode(RangingMode.SHORT)
            .withRegionOfInterest(8, 8, 4, 4)
            .withTimingBudget(TimingBudget.TIMING_BUDGET_33MS)
            .withThreshold(200);

        lc[1] = new LazyCan(3)
            .withRangingMode(RangingMode.SHORT)
            .withRegionOfInterest(8, 8, 4, 4)
            .withTimingBudget(TimingBudget.TIMING_BUDGET_33MS)
            .withThreshold(230);
    }

    public int[] getDistances() {
        distances[0] = lc[0].getDistanceMm();
        distances[1] = lc[1].getDistanceMm();
        return distances;
    }

    public boolean[] getStatuses() {
        statuses[0] = lc[0].getStatus();
        statuses[1] = lc[1].getStatus();
        return statuses;
    }

    public static LaserAlign system() {
        if (system == null) {
            system = new LaserAlign();
        }

        return system;
    }
}
