package frc.trigon.robot.subsystems.roller;

import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.subsystems.roller.toohardroller.ToohardRollerConstants;
import frc.trigon.robot.utilities.Commands;
import org.littletonrobotics.junction.Logger;

public class Roller extends SubsystemBase {
    private final static Roller INSTANCE = new Roller();
    private final RollerIO rollerIO = RollerIO.generateIO();
    private final RollerInputsAutoLogged rollerInputs = new RollerInputsAutoLogged();

    public static Roller getInstance() {
        return INSTANCE;
    }

    private Roller() {
    }

    @Override
    public void periodic() {
        rollerIO.updateInputs(rollerInputs);
        Logger.processInputs("Roller", rollerInputs);
    }

    /**
     * @return a command that only opens the angle of the roller
     */
    public Command getOpenRollerCommand() {
        return new FunctionalCommand(
                this::openRoller,
                () -> {
                },
                (interrupted) -> stopAngleMotor(),
                this::isOpen,
                this
        );
    }

    /**
     * @return a command that only closes the angle of the roller
     */
    public Command getCloseRollerCommand() {
        return new FunctionalCommand(
                this::closeRoller,
                () -> {
                },
                (interrupted) -> stopAngleMotor(),
                this::isClosed
        );
    }

    /**
     * @return a command that activates the collection motor with collection power
     */
    public Command getCollectCommand() {
        return new StartEndCommand(
                this::collect,
                this::stopCollection,
                this
        );
    }

    /**
     * @return a command that stops the collection motor
     */
    public Command getStopCollectionCommand() {
        return new InstantCommand(
                this::stopCollection,
                this
        );
    }

    /**
     * @return a command that opens the angle of the roller and activates the collection motor
     */
    public Command getFullOpeningCommand() {
        return new ParallelCommandGroup(
                getOpenRollerCommand(),
                Commands.removeRequirements(getCollectCommand())
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

    private void openRoller() {
        rollerIO.setAngleMotorPower(RollerConstants.OPEN_POWER);
    }

    private void closeRoller() {
        rollerIO.setAngleMotorPower(RollerConstants.CLOSE_POWER);
    }

    private void collect() {
        rollerIO.setCollectionMotorPower(RollerConstants.COLLECTION_MOTOR_SPEED);
    }

    private void stopCollection() {
        rollerIO.stopCollectionMotor();
    }

    private void stopAngleMotor() {
        rollerIO.stopAngleMotor();
    }

    private boolean isOpen() {
        return !ToohardRollerConstants.FORWARD_LIMIT_SWITCH.get();
    }

    private boolean isClosed() {
        return !ToohardRollerConstants.BACKWARD_LIMIT_SWITCH.get();
    }
}

