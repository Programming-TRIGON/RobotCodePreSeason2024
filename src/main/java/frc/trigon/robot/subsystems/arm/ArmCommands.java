package frc.trigon.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.Set;

public class ArmCommands {
    private static final Arm arm = Arm.getInstance();

    public static Command getSetTargetArmStateCommand(ArmConstants.ArmState targetState) {
        return getSetTargetArmStateCommand(targetState, 100, 100);
    }

    public static Command getSetTargetArmPositionCommand(Rotation2d targetAngle, double targetElevatorPosition) {
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
    public static Command getSetTargetArmStateCommand(ArmConstants.ArmState targetState, double angleSpeedPercentage, double elevatorSpeedPercentage) {
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
    public static Command getSetTargetArmPositionCommand(Rotation2d targetAngle, double targetElevatorPosition, double angleSpeedPercentage, double elevatorSpeedPercentage) {
        return new DeferredCommand(
                () -> getCurrentSetTargetArmPositionCommand(targetAngle, targetElevatorPosition, angleSpeedPercentage, elevatorSpeedPercentage),
                Set.of(arm)
        );
    }

    public static Command getCurrentSetTargetArmPositionCommand(Rotation2d targetAngle, double targetElevatorPosition, double angleSpeedPercentage, double elevatorSpeedPercentage) {
        if (isElevatorRising(targetElevatorPosition)) {
            return new SequentialCommandGroup(
                    getSetTargetAngleCommand(targetAngle, angleSpeedPercentage).until(arm.atTargetAngle(targetAngle)),
                    getSetTargetElevatorPositionCommand(targetElevatorPosition, elevatorSpeedPercentage)
            );
        }
        return new SequentialCommandGroup(
                getSetTargetElevatorPositionCommand(targetElevatorPosition, elevatorSpeedPercentage).until(arm.atTargetElevatorPosition(targetElevatorPosition)),
                getSetTargetAngleCommand(targetAngle, angleSpeedPercentage)
        );
    }

    private static Command getSetTargetAngleCommand(Rotation2d targetAngle, double speedPercentage) {
        return new FunctionalCommand(
                () -> arm.generateAngleMotorProfile(targetAngle, speedPercentage),
                arm::setTargetAngleFromProfile,
                (interrupted) -> {
                },
                () -> false,
                arm
        );
    }

    private static Command getSetTargetElevatorPositionCommand(double targetElevatorPosition, double speedPercentage) {
        return new FunctionalCommand(
                () -> arm.generateElevatorMotorProfile(targetElevatorPosition, speedPercentage),
                arm::setTargetElevatorPositionFromProfile,
                (interrupted) -> {
                },
                () -> false,
                arm
        );
    }

    private static boolean isElevatorRising(double targetElevatorPosition) {
        return targetElevatorPosition >= arm.getElevatorPositionRevolutions();
    }
}