package frc.trigon.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import java.util.Set;

public class ArmCommands {
    public static Command getSetTargetArmStateCommand(ArmConstants.ArmState targetState) {
        return getSetTargetArmStateCommand(targetState, 100, 100);
    }

    public static Command getSetTargetArmPositionCommand(Rotation2d angle, double elevatorPosition) {
        return getSetTargetArmPositionCommand(angle, elevatorPosition, 100, 100);
    }

    /**
     * Creates a command that sets the target state of the arm.
     *
     * @param targetState             the target state of the arm
     * @param angleSpeedPercentage    the percentage of speed that the angle will move in
     * @param elevatorSpeedPercentage the percentage of speed that the elevator will move in
     * @return the command
     */
    public static Command getSetTargetArmStateCommand(ArmConstants.ArmState targetState, double angleSpeedPercentage, double elevatorSpeedPercentage) {
        return getSetTargetArmPositionCommand(targetState.angle, targetState.elevatorPosition, angleSpeedPercentage, elevatorSpeedPercentage);
    }

    /**
     * Creates a command that sets the target position of the arm.
     *
     * @param targetAngle             the target angle of the arm
     * @param targetElevatorPosition  the target position of the arm's elevator
     * @param angleSpeedPercentage    the percentage of speed that the angle will move in
     * @param elevatorSpeedPercentage the percentage of speed that the elevator will move in
     * @return the command
     */
    public static Command getSetTargetArmPositionCommand(Rotation2d targetAngle, double targetElevatorPosition, double angleSpeedPercentage, double elevatorSpeedPercentage) {
        return new DeferredCommand(
                () -> getCurrentSetTargetArmStateCommand(targetAngle, targetElevatorPosition, angleSpeedPercentage, elevatorSpeedPercentage),
                Set.of(Arm.getInstance())
        );
    }

    private static Command getCurrentSetTargetArmStateCommand(Rotation2d angle, double elevatorPosition, double angleSpeedPercentage, double elevatorSpeedPercentage) {
        if (Arm.getInstance().isElevatorOpening(elevatorPosition)) {
            return new SequentialCommandGroup(
                    getSetTargetAngleCommand(angle, angleSpeedPercentage),
                    getSetTargetElevatorPositionCommand(elevatorPosition, elevatorSpeedPercentage)
            );
        }
        return new SequentialCommandGroup(
                getSetTargetElevatorPositionCommand(elevatorPosition, elevatorSpeedPercentage),
                getSetTargetAngleCommand(angle, angleSpeedPercentage)
        );
    }

    private static Command getSetTargetAngleCommand(Rotation2d targetAngle, double speedPercentage) {
        return new FunctionalCommand(
                () -> Arm.getInstance().generateAngleMotorProfile(targetAngle, speedPercentage),
                () -> Arm.getInstance().setTargetAngleFromProfile(),
                (interrupted) -> {
                },
                () -> false,
                Arm.getInstance()
        );
    }

    private static Command getSetTargetElevatorPositionCommand(double targetElevatorPosition, double speedPercentage) {
        return new FunctionalCommand(
                () -> Arm.getInstance().generateElevatorMotorProfile(targetElevatorPosition, speedPercentage),
                () -> Arm.getInstance().setTargetElevatorPositionFromProfile(),
                (interrupted) -> {
                },
                () -> false,
                Arm.getInstance()
        );
    }
}
