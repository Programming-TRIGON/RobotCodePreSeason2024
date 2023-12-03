package frc.trigon.robot.subsystems.collector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class CollectorCommands {
    public static final Collector collector = Collector.getInstance();
    public static Command getSetTargetStateCommand(CollectorConstants.CollectorStates targetState) {
        return new StartEndCommand(
                () -> collector.setPower(targetState.power),
                () -> {
                },
                collector
        );
    }
}
