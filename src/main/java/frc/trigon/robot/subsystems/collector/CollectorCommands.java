package frc.trigon.robot.subsystems.collector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class CollectorCommands {
    public static final Collector COLLECTOR = Collector.getInstance();
    
    public static Command getSetTargetStateCommand(CollectorConstants.CollectorStates targetState) {
        return new StartEndCommand(
                () -> COLLECTOR.setPower(targetState.power),
                () -> {
                },
                COLLECTOR
        );
    }
}
