package frc.trigon.robot.subsystems.roller;

import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.utilities.Commands;

public class RollerCommands {
    private static final Roller roller = Roller.getInstance();
    /**
     * @return a command that only opens the angle of the roller
     */
    public static Command getOpenRollerCommand() {
        return new FunctionalCommand(
                roller::openRoller,
                () -> {
                },
                (interrupted) -> roller.stopAngleMotor(),
                roller::isOpen,
                roller
        );
    }

    /**
     * @return a command that only closes the angle of the roller
     */
    public static Command getCloseRollerCommand() {
        return new FunctionalCommand(
                roller::closeRoller,
                () -> {
                },
                (interrupted) -> roller.stopAngleMotor(),
                roller::isClosed
        );
    }

    /**
     * @return a command that activates the collection motor with collection power
     */
    public static Command getCollectCommand() {
        return new StartEndCommand(
                roller::collect,
                roller::stopCollectionMotor,
                roller
        );
    }

    /**
     * @return a command that stops the collection motor
     */
    public static Command getStopCollectionCommand() {
        return new InstantCommand(
                roller::stopCollectionMotor,
                roller
        );
    }

    /**
     * @return a command that opens the angle of the roller and activates the collection motor
     */
    public static Command getFullOpeningCommand() {
        return new ParallelCommandGroup(
                getOpenRollerCommand(),
                frc.trigon.robot.utilities.Commands.removeRequirements(getCollectCommand())
        );
    }

    /**
     * @return a command that closes the angle of the roller and stops the collection motor
     */
    public static Command getFullStopCommand() {
        return new ParallelCommandGroup(
                getCloseRollerCommand(),
                Commands.removeRequirements(getStopCollectionCommand())
        );
    }
}
