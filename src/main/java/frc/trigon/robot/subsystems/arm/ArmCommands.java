package frc.trigon.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.Set;

public class ArmCommands {
    private final ArmIO armIO = ArmIO.generateIO();
    private final ArmInputsAutoLogged armInputs = new ArmInputsAutoLogged();
    public Command getSetTargetArmStateCommand(ArmConstants.ArmState targetState) {
        return getSetTargetArmStateCommand(targetState, 100, 100);
    }

    public Command getSetTargetArmPositionCommand(Rotation2d targetAngle, double targetElevatorPosition) {
        return getSetTargetArmPositionCommand(targetAngle, targetElevatorPosition, 100, 100);
    }

    /**
     * Creates a command that sets the target state of the arm.
     *
     * @param targetState             the target state of the arm
     * @param angleSpeedPercentage    the target speed for the angle motor as a percentage of the normal speed
     * @param elevatorSpeedPercentage the target speed for the elevator motor as a percentage of the normal speed
     * @return the command
     */
    public Command getSetTargetArmStateCommand(ArmConstants.ArmState targetState, double angleSpeedPercentage, double elevatorSpeedPercentage) {
        return getSetTargetArmPositionCommand(targetState.angle, targetState.elevatorPosition, angleSpeedPercentage, elevatorSpeedPercentage);
    }

    /**
     * Creates a command that sets the target position of the arm.
     *
     * @param targetAngle             the target angle of the arm
     * @param targetElevatorPosition  the target elevator position
     * @param angleSpeedPercentage    the target speed for angle motor as a percentage of normal speed
     * @param elevatorSpeedPercentage the target speed for elevator motor as a percentage of normal speed
     * @return the command
     */
    public Command getSetTargetArmPositionCommand(Rotation2d targetAngle, double targetElevatorPosition, double angleSpeedPercentage, double elevatorSpeedPercentage) {
        return new DeferredCommand(
                () -> getCurrentSetTargetArmPositionCommand(targetAngle, targetElevatorPosition, angleSpeedPercentage, elevatorSpeedPercentage),
                Set.of(Arm.getInstance())
        );
    }

    public Command getCurrentSetTargetArmPositionCommand(Rotation2d targetAngle, double targetElevatorPosition, double angleSpeedPercentage, double elevatorSpeedPercentage) {
        if (targetElevatorPosition >= armInputs.elevatorPositionRevolutions) {
            return new SequentialCommandGroup(
                    getSetTargetAngleCommand(targetAngle, angleSpeedPercentage),
                    getSetTargetElevatorPositionCommand(targetElevatorPosition, elevatorSpeedPercentage)
            );
        }
        return new SequentialCommandGroup(
                getSetTargetElevatorPositionCommand(targetElevatorPosition, elevatorSpeedPercentage),
                getSetTargetAngleCommand(targetAngle, angleSpeedPercentage)
        );
    }

    private Command getSetTargetAngleCommand(Rotation2d targetAngle, double speedPercentage) {
        return new FunctionalCommand(
                () -> Arm.getInstance().generateAngleMotorProfile(targetAngle, speedPercentage),
                Arm.getInstance()::setTargetAngleFromProfile,
                (interrupted) -> {
                },
                () -> false,
                Arm.getInstance()
        );
    }

    private Command getSetTargetElevatorPositionCommand(double targetElevatorPosition, double speedPercentage) {
        return new FunctionalCommand(
                () -> Arm.getInstance().generateElevatorMotorProfile(targetElevatorPosition, speedPercentage),
                Arm.getInstance()::setTargetElevatorPositionFromProfile,
                (interrupted) -> {
                },
                () -> false,
                Arm.getInstance()
        );
    }
}
