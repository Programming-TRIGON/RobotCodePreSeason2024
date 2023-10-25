package frc.trigon.robot.subsystems.collector;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CollectorSubsystem extends SubsystemBase {
    private final static CollectorSubsystem INSTANCE = new CollectorSubsystem();
    private final TalonSRX motor = CollectorConstants.MOTOR;
    // motor.getSupplyCurrent

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