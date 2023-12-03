package frc.trigon.robot.subsystems.roller;

import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.utilities.Commands;

public class RollerCommands {
    /**
     * @return a command that only opens the angle of the roller
     */
    public Command getOpenRollerCommand() {
        return new FunctionalCommand(
                Roller.getInstance()::openRoller,
                () -> {
                },
                (interrupted) -> Roller.getInstance().stopAngleMotor(),
                Roller.getInstance()::isOpen,
                Roller.getInstance()
        );
    }

    /**
     * @return a command that only closes the angle of the roller
     */
    public Command getCloseRollerCommand() {
        return new FunctionalCommand(
                Roller.getInstance()::closeRoller,
                () -> {
                },
                (interrupted) -> Roller.getInstance().stopAngleMotor(),
                Roller.getInstance()::isClosed
        );
    }

    /**
     * @return a command that activates the collection motor with collection power
     */
    public Command getCollectCommand() {
        return new StartEndCommand(
                Roller.getInstance()::collect,
                Roller.getInstance()::stopCollectionMotor,
                Roller.getInstance()
        );
    }

    /**
     * @return a command that stops the collection motor
     */
    public Command getStopCollectionCommand() {
        return new InstantCommand(
                Roller.getInstance()::stopCollectionMotor,
                Roller.getInstance()
        );
    }

    /**
     * @return a command that opens the angle of the roller and activates the collection motor
     */
    public Command getFullOpeningCommand() {
        return new ParallelCommandGroup(
                getOpenRollerCommand(),
                frc.trigon.robot.utilities.Commands.removeRequirements(getCollectCommand())
        );
    }

    /**
     * @return a command that closes the angle of the roller and stops the collection motor
     */
    public Command getFullStopCommand() {
        return new ParallelCommandGroup(
                getCloseRollerCommand(),
                Commands.removeRequirements(getStopCollectionCommand())
        );
    }
}
