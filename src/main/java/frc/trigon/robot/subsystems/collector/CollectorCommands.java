package frc.trigon.robot.subsystems.collector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class CollectorCommands {
    public static Command getSetTargetStateCommand(CollectorConstants.CollectorStates state) {
        return new StartEndCommand(
                () -> Collector.getInstance().setPower(state.power),
                () -> {
                },
                Collector.getInstance()
        );
    }
}
