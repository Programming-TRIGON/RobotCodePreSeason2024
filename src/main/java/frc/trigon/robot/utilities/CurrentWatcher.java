package frc.trigon.robot.utilities;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.NotifierCommand;

import java.util.function.Supplier;

public class CurrentWatcher {
    private double previousTime;

    public void currentWatcher(Runnable runnable, Supplier<Integer> currentSupplier, int maxCurrent, Supplier<Double> timeThreshold){
        Notifier notifier = new Notifier(runnable);
        notifier.startPeriodic(0.02);
        if (currentSupplier.get() < maxCurrent){
            this.previousTime = timeThreshold.get();
            return;
        }
        double currentTime = timeThreshold.get();
        double difference = currentTime - previousTime;

    }
}
