package frc.trigon.robot.subsystems.collector;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

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
     * creates a command that gives power to the motor according to the inputted state.
     * @param state the inputted state
     * @return the command
     */
    public CommandBase setCollectorStateCommand(CollectorConstants.CollectorStates state){
        return new FunctionalCommand(
                () -> {},
                () -> motor.set(ControlMode.PercentOutput, state.power),
                (interrupted) -> stop(),
                () -> false,
                this
        );
    }

    private void stop(){
        motor.set(ControlMode.Disabled, 0);
    }
}

