package frc.trigon.robot.utilities;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class Commands {
    public static CommandBase removeRequirements(CommandBase command) {
        return new FunctionalCommand(
                command::initialize,
                command::execute,
                command::end,
                command::isFinished
        );
    }
}
