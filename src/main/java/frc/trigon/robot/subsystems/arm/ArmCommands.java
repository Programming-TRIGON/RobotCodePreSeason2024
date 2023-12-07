package frc.trigon.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.utilities.Commands;

import java.util.Set;

public class ArmCommands {
    private static final Arm ARM = Arm.getInstance();

    public static Command getSetTargetArmStateCommand(ArmConstants.ArmState targetState) {
        return getSetTargetArmStateCommand(targetState, 100, 100);
    }

    public static Command getSetTargetArmPositionCommand(Rotation2d targetAngle, double targetElevatorPositionMeters) {
        return getSetTargetArmPositionCommand(targetAngle, targetElevatorPositionMeters, 100, 100);
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
        return getSetTargetArmPositionCommand(targetState.angle, targetState.elevatorPositionMeters, angleSpeedPercentage, elevatorSpeedPercentage);
    }

    /**
     * Creates a command that sets the target position of the arm.
     *
     * @param targetAngle                  the target angle of the arm
     * @param targetElevatorPositionMeters the target elevator position
     * @param angleSpeedPercentage         the target speed for angle motor as a percentage of normal speed
     * @param elevatorSpeedPercentage      the target speed for elevator motor as a percentage of normal speed
     * @return the command
     */
    public static Command getSetTargetArmPositionCommand(Rotation2d targetAngle, double targetElevatorPositionMeters, double angleSpeedPercentage, double elevatorSpeedPercentage) {
        return new DeferredCommand(
                () -> getCurrentSetTargetArmPositionCommand(targetAngle, targetElevatorPositionMeters, angleSpeedPercentage, elevatorSpeedPercentage),
                Set.of(ARM)
        );
    }

    private static Command getCurrentSetTargetArmPositionCommand(Rotation2d targetAngle, double targetElevatorPositionMeters, double angleSpeedPercentage, double elevatorSpeedPercentage) {
        if (ARM.isElevatorRising(targetElevatorPositionMeters)) {
            return new SequentialCommandGroup(
                    getSetTargetAngleCommand(targetAngle, angleSpeedPercentage).until(() -> ARM.atAngle(targetAngle)),
                    getSetTargetElevatorPositionCommand(targetElevatorPositionMeters, elevatorSpeedPercentage).alongWith(
                            Commands.removeRequirements(getSetTargetAngleCommand(targetAngle, angleSpeedPercentage))
                    )
            );
        }
        return new SequentialCommandGroup(
                getSetTargetElevatorPositionCommand(targetElevatorPositionMeters, elevatorSpeedPercentage).until(() -> ARM.atElevatorPositionMeters(targetElevatorPositionMeters)),
                getSetTargetAngleCommand(targetAngle, angleSpeedPercentage).alongWith(
                        Commands.removeRequirements(getSetTargetElevatorPositionCommand(targetElevatorPositionMeters, elevatorSpeedPercentage))
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