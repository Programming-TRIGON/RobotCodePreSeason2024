package frc.trigon.robot.subsystems.roller;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.utilities.Commands;

public class Roller extends SubsystemBase {
    private final static Roller INSTANCE = new Roller();
    private final TalonSRX angleMotor = RollerConstants.ANGLE_MOTOR;
    private final CANSparkMax collectionMotor = RollerConstants.COLLECTION_MOTOR;

    public static Roller getInstance() {
        return INSTANCE;
    }

    private Roller() {
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
        angleMotor.set(ControlMode.PercentOutput, RollerConstants.OPEN_POWER);
    }

    private void closeRoller() {
        angleMotor.set(ControlMode.PercentOutput, RollerConstants.CLOSE_POWER);
    }

    private void collect() {
        collectionMotor.set(RollerConstants.COLLECTION_MOTOR_SPEED);
    }

    private void stopCollection() {
        collectionMotor.stopMotor();
    }

    private void stopAngleMotor() {
        angleMotor.set(ControlMode.Disabled, 0);
    }

    private boolean isOpen() {
        return !RollerConstants.FORWARD_LIMIT_SWITCH.get();
    }

    private boolean isClosed() {
        return !RollerConstants.BACKWARD_LIMIT_SWITCH.get();
    }
}

