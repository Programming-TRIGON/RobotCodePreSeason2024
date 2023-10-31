package frc.trigon.robot.utilities;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.NotifierCommand;

import java.util.function.Supplier;

public class CurrentWatcher {
    private double previousTime;

    public void currentWatcher(Runnable runnable, Supplier<Integer> current, int maxCurrent, Supplier<Double> time){
        Notifier notifier = new Notifier(runnable);
        notifier.startPeriodic(0.02);
        if (current.get() < maxCurrent){
            this.previousTime = time.get();
            return;
        }
        double currentTime = time.get();
        double difference = currentTime - previousTime;

    }
}
