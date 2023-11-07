package frc.trigon.robot.subsystems.collector;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.utilities.CurrentWatcher;

public class CollectorSubsystem extends SubsystemBase {
    private final static CollectorSubsystem INSTANCE = new CollectorSubsystem();
    private final TalonSRX motor = CollectorConstants.MOTOR;
    private final CurrentWatcher currentWatcher = new CurrentWatcher(
            this::stop,
            motor::getSupplyCurrent,
            CollectorConstants.MAX_CURRENT,
            CollectorConstants.MAX_CURRENT_TIME
    );

    public static CollectorSubsystem getInstance() {
        return INSTANCE;
    }

    private CollectorSubsystem() {
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
}