package frc.trigon.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.Set;

public class ArmCommands {
    private static final Arm ARM = Arm.getInstance();

    public static Command getSetTargetStateCommand(ArmConstants.ArmState state) {
        return getSetTargetPositionCommand(state.angle, state.elevatorPositionMeters, 100, 100);
    }

    /**
     * set the arm to the wanted state
     *
     * @param state                   the wanted state
     * @param elevatorSpeedPercentage max speed of the elevator in percentages
     * @param angleSpeedPercentage    max speed of the elevator angle in percentages
     * @return a command
     */
    public static Command getsetTargetStateCommand(ArmConstants.ArmState state, double elevatorSpeedPercentage, double angleSpeedPercentage) {
        return getSetTargetPositionCommand(state.angle, state.elevatorPositionMeters, elevatorSpeedPercentage, angleSpeedPercentage);
    }

    public static Command getSetTargetPositionCommand(Rotation2d targetAngle, double targetElevatorPositionMeters) {
        return getSetTargetPositionCommand(targetAngle, targetElevatorPositionMeters, 100, 100);
    }

    /**
     * set the arm to the wanted position
     *
     * @param targetAngle                  the target angle
     * @param targetElevatorPositionMeters the target of the elevator
     * @param elevatorSpeedPercentage      max speed of the elevator in percentages
     * @param angleSpeedPercentage         max speed of the elevator angle in percentages
     * @return a command
     */
    public static Command getSetTargetPositionCommand(Rotation2d targetAngle, double targetElevatorPositionMeters, double elevatorSpeedPercentage, double angleSpeedPercentage) {
        return new DeferredCommand(() -> getCurrentSetTargetPositionCommand(targetAngle, targetElevatorPositionMeters, elevatorSpeedPercentage, angleSpeedPercentage), Set.of());
    }

    private static Command getCurrentSetTargetPositionCommand(Rotation2d targetAngle, double targetElevatorPositionMeters, double elevatorSpeedPercentage, double angleSpeedPercentage) {
        if (ARM.isElevatorOpening(targetElevatorPositionMeters)) {
            return new SequentialCommandGroup(
                    getSetTargetAngleCommand(targetAngle, angleSpeedPercentage).until(() -> ARM.atAngle(targetAngle)),
                    getSetTargetElevatorPositionCommand(targetElevatorPositionMeters, elevatorSpeedPercentage)
            );
        }
        return new SequentialCommandGroup(
                getSetTargetElevatorPositionCommand(targetElevatorPositionMeters, elevatorSpeedPercentage).until(() -> ARM.atTargetPositionElevator(targetElevatorPositionMeters)),
                getSetTargetAngleCommand(targetAngle, angleSpeedPercentage)
        );
    }

    private static Command getSetTargetAngleCommand(Rotation2d targetAngle, double SpeedPercentage) {
        return new FunctionalCommand(
                () -> ARM.generateAngleMotorProfile(targetAngle, SpeedPercentage),
                ARM::setTargetAngleFromProfile,
                (interrupted) -> {
                },
                () -> false,
                ARM
        );
    }

    private static Command getSetTargetElevatorPositionCommand(double targetElevatorPositionMeters, double SpeedPercentage) {
        return new FunctionalCommand(
                () -> ARM.generateElevatorMotorProfile(targetElevatorPositionMeters, SpeedPercentage),
                ARM::setTargetElevatorPositionFromProfile,
                (interrupted) -> {
                },
                () -> false,
                ARM
        );
    }
}
