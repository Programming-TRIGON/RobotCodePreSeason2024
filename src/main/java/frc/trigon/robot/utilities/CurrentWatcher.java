package frc.trigon.robot.utilities;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

import java.util.function.Supplier;

/**
 * A class that checks if the motor's current passed a certain current limit for a certain amount of time. If so, runs a runnable.
 */
public class CurrentWatcher {
    private final Runnable runnable;
    private final Supplier<Integer> currentSupplier;
    private final int maxCurrent;
    private final double maxTime;
    private double belowCurrentTimeStamp;

    /**
     * Constructs a new CurrentWatcher that checks the current every 0.02 seconds, and if the current passes the current limit runs a runnable.
     *
     * @param runnable        the runnable to run if the current passes it's limit
     * @param currentSupplier a supplier for the motor's current
     * @param maxCurrent      the current limit
     * @param maxTime         the time needed for the current to exceed its limit, in order to run the runnable
     */
    public CurrentWatcher(Runnable runnable, Supplier<Integer> currentSupplier, int maxCurrent, double maxTime) {
        this.runnable = runnable;
        this.currentSupplier = currentSupplier;
        this.maxCurrent = maxCurrent;
        this.maxTime = maxTime;

        new Notifier(this::checkCurrent).startPeriodic(0.02);
    }

    private void checkCurrent() {
        if (isBelowCurrentLimit()) {
            belowCurrentTimeStamp = Timer.getFPGATimestamp();
            return;
        }

        if (didPassTimeThreshold())
            runnable.run();
    }

    private boolean isBelowCurrentLimit() {
        return currentSupplier.get() <= maxCurrent;
    }

    private boolean didPassTimeThreshold() {
        double timeDifference = Timer.getFPGATimestamp() - belowCurrentTimeStamp;
        return maxTime > timeDifference;
    }
}
