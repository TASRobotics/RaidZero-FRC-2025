package raidzero.robot.test;

import edu.wpi.first.hal.simulation.DriverStationDataJNI;
import edu.wpi.first.wpilibj.DriverStation;
import raidzero.robot.Robot;

public class AutomatedTestRobot extends Robot {
    private static AutomatedTestRobot instance = null;

    /**
     * Constructs a new {@link AutomatedTestRobot} instance
     */
    private AutomatedTestRobot() {
        System.out.println("Robot type: Automated test");
    }

    @Override
    public void startCompetition() {
        try {
            super.startCompetition();
        } catch (Throwable throwable) {
            Throwable cause = throwable.getCause();

            if (cause != null) {
                throwable = cause;
            }

            DriverStation.reportError(
                "Unhandled exception: " + throwable.toString(), throwable.getStackTrace()
            );

            System.exit(-1);
        }
    }

    @Override
    public void robotInit() {
        super.robotInit();

        new Thread(this::runTest).start();
    }

    /**
     * Runs the test
     */
    private void runTest() {
        System.out.println("Waiting two seconds for robot to finish startup");
        sleep(2000);

        System.out.println("Enabling autonomous mode and waiting 10 seconds");
        DriverStationDataJNI.setAutonomous(true);
        DriverStationDataJNI.setEnabled(true);

        sleep(10000);

        System.out.println("Disabling robot and waiting two seconds");
        DriverStationDataJNI.setEnabled(false);

        sleep(2000);

        System.out.println("Ending competition");
        suppressExitWarning(true);
        endCompetition();
    }

    /**
     * Sleeps the thread for the specified duration
     *
     * @param durationMillis The duration to sleep in milliseconds
     */
    private static void sleep(long durationMillis) {
        try {
            Thread.sleep(durationMillis);
        } catch (InterruptedException interrupt) {
            System.out.println("Interrupted");
        }
    }

    /**
     * Gets the instance of the {@link AutomatedTestRobot}
     *
     * @return The instance of the {@link AutomatedTestRobot}
     */
    public static AutomatedTestRobot getInstance() {
        if (instance == null) {
            instance = new AutomatedTestRobot();
        }

        return instance;
    }
}
