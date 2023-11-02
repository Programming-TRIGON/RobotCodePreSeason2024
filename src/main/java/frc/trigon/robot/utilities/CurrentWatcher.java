package frc.trigon.robot.utilities;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

import java.util.function.Supplier;

/**
 * A class that checks if the current past a certain limit for a certain amount of time.
 */
public class CurrentWatcher {
    private double bellowCurrentTime;
    private final Runnable runnable;
    private final Supplier<Integer> currentSupplier;
    private final int maxCurrent;
    private final double timeThreshold;

    /**
     * Checks the current every 0.02 seconds, and it passes runs a runnable.
     *
     * @param runnable        what will run if the current passes it's limit
     * @param currentSupplier the current
     * @param maxCurrent      the current limit
     * @param timeThreshold   the time
     */
    public CurrentWatcher(Runnable runnable, Supplier<Integer> currentSupplier, int maxCurrent, double timeThreshold) {
        this.runnable = runnable;
        this.currentSupplier = currentSupplier;
        this.maxCurrent = maxCurrent;
        this.timeThreshold = timeThreshold;

        new Notifier(this::checkCurrent).startPeriodic(0.02);
    }

    private void checkCurrent() {
        if (isBellowCurrentLimit()) {
            bellowCurrentTime = Timer.getFPGATimestamp();
            return;
        }
        if (didPassTimeThreshold())
            runnable.run();
    }

    private boolean isBellowCurrentLimit() {
        return currentSupplier.get() <= maxCurrent;
    }

    private boolean didPassTimeThreshold() {
        double timeDifference = Timer.getFPGATimestamp() - bellowCurrentTime;
        return timeThreshold > timeDifference;
    }


}
