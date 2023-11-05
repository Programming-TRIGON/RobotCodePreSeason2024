package frc.trigon.robot.subsystems.collector;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.utilities.CurrentWatcher;
import java.util.function.Supplier;

public class CollectorSubsystem extends SubsystemBase {
    private final static CollectorSubsystem INSTANCE = new CollectorSubsystem();
    private final TalonSRX motor = CollectorConstants.MOTOR;
    private final Supplier<Double> motorCurrent = motor::getSupplyCurrent;
    private final CurrentWatcher currentWatcher = new CurrentWatcher(
            this::stop,
            motorCurrent,
            CollectorConstants.MAX_CURRENT,
            CollectorConstants.TIME_THRESHOLD
    );

    public static CollectorSubsystem getInstance() {
        return INSTANCE;
    }

    private CollectorSubsystem() {
    }

    /**
     * Creates a command that gives power to the motor according to the target state.
     *
     * @param state the target state
     * @return the command
     */
    public CommandBase setCollectorStateCommand(CollectorConstants.CollectorStates state) {
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