package frc.trigon.robot.subsystems.collector;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.utilities.CurrentWatcher;

public class Collector extends SubsystemBase {
    private final static Collector INSTANCE = new Collector();
    private final TalonSRX motor = CollectorConstants.MOTOR;
    private final CurrentWatcher currentWatcher = new CurrentWatcher(
            this::stop,
            motor::getSupplyCurrent,
            CollectorConstants.MAX_CURRENT,
            CollectorConstants.MAX_CURRENT_TIME
    );

    public static Collector getInstance() {
        return INSTANCE;
    }

    private Collector() {
        startCurrentWatcher();
    }

    public Command getSetTargetStateCommand(CollectorConstants.CollectorStates state) {
        return new StartEndCommand(
                () -> motor.set(ControlMode.PercentOutput, state.power),
                () -> {},
                this
        );
    }

    private void stop() {
        motor.set(ControlMode.Disabled, 0);
    }

    private void startCurrentWatcher(){
        new CurrentWatcher(
                this::stop,
                motor::getSupplyCurrent,
                CollectorConstants.MAX_CURRENT,
                CollectorConstants.MAX_CURRENT_TIME
        );
    }
}