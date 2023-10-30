package frc.trigon.robot.utilities;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.NotifierCommand;

import java.util.function.Supplier;

public class CurrentWatcher {
    double prevTime;

    public void currentWatcher(Runnable runnable, Supplier<Integer> current, Supplier<Integer> maxAmount, Supplier<Double> time){
        if (current.get() < maxAmount.get()){
            this.prevTime = time.get();
            return;
        }
        double currentTime = time.get();
        double difference = currentTime - prevTime;
        
    }
}
