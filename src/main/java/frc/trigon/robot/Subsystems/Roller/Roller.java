package frc.trigon.robot.Subsystems.Roller;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.*;

public class Roller extends SubsystemBase {
    private final static Roller INSTANCE = new Roller();
    private final TalonSRX angleMotor = RollerConstants.ANGLE_MOTOR;
    private final CANSparkMax collectionMotor = RollerConstants.COLLECTION_MOTOR;

    public static Roller getInstance() {
        return INSTANCE;
    }

    private Roller() {
    }

    public CommandBase getOpenRollerCommand()   {
        return new FunctionalCommand(
                this::openRoller,
                ()-> {},
                (interrupted)-> stopAngle(),
                this::isOpen
        );
    }
    public CommandBase getCloseRollerCommand()  {
        return new FunctionalCommand(
                this::closeRoller,
                ()->{},
                (interrupted)-> stopAngle(),
                this::isClosed
        );
    }
    public CommandBase getCollectCommand() {
        return new StartEndCommand(
                this::collect,
                this::stopCollect
        );
    }
    public CommandBase getStopCollectCommand()  {
        return new InstantCommand(
                this::stopCollect
        );
    }

    public CommandBase fullOpeningCommand() {
        return new ParallelCommandGroup(
                getOpenRollerCommand(),
                getCollectCommand()
        );
    }
    public CommandBase fullStopCommand()    {
        return new ParallelCommandGroup(
                getCloseRollerCommand(),
                getStopCollectCommand()
        );
    }
    private void openRoller()    {
        angleMotor.set(ControlMode.PercentOutput, RollerConstants.OPEN_POWER);
    }

    private void closeRoller()   {
        angleMotor.set(ControlMode.PercentOutput, RollerConstants.CLOSE_POWER);
    }

    private void collect()  {
        collectionMotor.set(RollerConstants.COLLECTION_MOTOR_SPEED);
    }

    private void stopCollect()  {
        collectionMotor.stopMotor();
    }

    private void stopAngle() {
        angleMotor.set(ControlMode.Disabled, 0);
    }

    private boolean isOpen()   {
        return !RollerConstants.FORWARD_LIMIT_SWITCH.get();
    }

    private boolean isClosed() {
        return !RollerConstants.BACKWARD_LIMIT_SWITCH.get();
    }

}

