package frc.trigon.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.utilities.Commands;

import java.util.Set;

public class ArmCommands {
    private static final Arm ARM = Arm.getInstance();

    public static Command getSetTargetArmStateCommand(ArmConstants.ArmState targetState) {
        return getSetTargetArmStateCommand(targetState, 100, 100);
    }

    public static Command getSetTargetArmPositionCommand(Rotation2d targetAngle, double targetElevatorPositionMeter) {
        return getSetTargetArmPositionCommand(targetAngle, targetElevatorPositionMeter, 100, 100);
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
        return getSetTargetArmPositionCommand(targetState.angle, targetState.elevatorPositionMeters, angleSpeedPercentage, elevatorSpeedPercentage);
    }

    /**
     * Creates a command that sets the target position of the arm.
     *
     * @param targetAngle             the target angle of the arm
     * @param targetElevatorPositionMeters  the target position of the arm's elevator
     * @param angleSpeedPercentage    the percentage of speed that the angle will move in
     * @param elevatorSpeedPercentage the percentage of speed that the elevator will move in
     * @return the command
     */
    public static Command getSetTargetArmPositionCommand(Rotation2d targetAngle, double targetElevatorPositionMeters, double angleSpeedPercentage, double elevatorSpeedPercentage) {
        return new DeferredCommand(
                () -> getCurrentSetTargetArmStateCommand(targetAngle, targetElevatorPositionMeters, angleSpeedPercentage, elevatorSpeedPercentage),
                Set.of(ARM)
        );
    }

    private static Command getCurrentSetTargetArmStateCommand(Rotation2d targetAngle, double elevatorPosition, double angleSpeedPercentage, double elevatorSpeedPercentageMeters) {
        if (ARM.isElevatorOpening(elevatorPosition)) {
            return new SequentialCommandGroup(
                    getSetTargetAngleCommand(targetAngle, angleSpeedPercentage).until(() -> ARM.atAngle(targetAngle)),
                    getSetTargetElevatorPositionCommand(elevatorPosition, elevatorSpeedPercentageMeters).alongWith(
                            Commands.removeRequirements(getSetTargetAngleCommand(targetAngle, angleSpeedPercentage))
                    )
            );
        }
        return new SequentialCommandGroup(
                getSetTargetElevatorPositionCommand(elevatorPosition, elevatorSpeedPercentageMeters).until(() -> ARM.atTargetElevatorPosition(elevatorPosition)),
                getSetTargetAngleCommand(targetAngle, angleSpeedPercentage).alongWith(
                        Commands.removeRequirements(getSetTargetElevatorPositionCommand(elevatorPosition, elevatorSpeedPercentageMeters))
                )
        );
    }

    private static Command getSetTargetAngleCommand(Rotation2d targetAngle, double speedPercentage) {
        return new FunctionalCommand(
                () -> ARM.generateAngleMotorProfile(targetAngle, speedPercentage),
                ARM::setTargetAngleFromProfile,
                (interrupted) -> {
                },
                () -> false,
                ARM
        );
    }

    private static Command getSetTargetElevatorPositionCommand(double targetElevatorPositionMeters, double speedPercentage) {
        return new FunctionalCommand(
                () -> ARM.generateElevatorMotorProfile(targetElevatorPositionMeters, speedPercentage),
                ARM::setTargetElevatorPositionFromProfile,
                (interrupted) -> {
                },
                () -> false,
                ARM
        );
    }
}
