package frc.trigon.robot.utilities;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

import java.util.function.Supplier;

/**
 * A clas that checks if the current passed a certain current limit for a certain amount of time and if so, runs a runnable.
 */
public class CurrentWatcher {
    private final Runnable runnable;
    private final Supplier<Integer> currentSupplier;
    private final int maxCurrent;
    private final double timeThreshold;
    private double bellowCurrentTime;

    /**
     * Construct a new CurrentWatcher that will Check the current every 0.02 seconds, and if the current passes runs a runnable.
     *
     * @param runnable        what will run if the current passes it's limit
     * @param currentSupplier a supplier for the motor's current
     * @param maxCurrent      the current limit
     * @param timeThreshold   the time since the motor was started
     */
    public CurrentWatcher(Runnable runnable, Supplier<Integer> currentSupplier, int maxCurrent, double timeThreshold) {
        this.runnable = runnable;
        this.currentSupplier = currentSupplier;
        this.maxCurrent = maxCurrent;
        this.timeThreshold = timeThreshold;

        new Notifier(this::checkCurrent).startPeriodic(0.02);
    }

    private void checkCurrent() {
        if (isBelowCurrentLimit()) {
            bellowCurrentTime = Timer.getFPGATimestamp();
            return;
        }
        
        if (didPassTimeThreshold())
            runnable.run();
    }

    private boolean isBelowCurrentLimit() {
        return currentSupplier.get() <= maxCurrent;
    }

    private boolean didPassTimeThreshold() {
        double timeDifference = Timer.getFPGATimestamp() - bellowCurrentTime;
        return timeThreshold > timeDifference;
    }
}
