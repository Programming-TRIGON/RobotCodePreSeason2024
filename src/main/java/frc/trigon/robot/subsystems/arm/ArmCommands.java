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

    public static Command getsetTargetStateCommand(ArmConstants.ArmState state, double elevatorSpeedPercentage, double angleSpeedPercentage) {
        return getSetTargetPositionCommand(state.angle, state.elevatorPositionMeters, elevatorSpeedPercentage, angleSpeedPercentage);
    }

    public static Command getSetTargetPositionCommand(Rotation2d targetAngle, double targetElevatorPositionMeters) {
        return getSetTargetPositionCommand(targetAngle, targetElevatorPositionMeters, 100, 100);
    }

    public static Command getSetTargetPositionCommand(Rotation2d targetAngle, double targetElevatorPositionMeters, double elevatorSpeedPercentage, double angleSpeedPercentage) {
        return new DeferredCommand(() -> getSetCurrentTargetPositionCommand(targetAngle, targetElevatorPositionMeters, elevatorSpeedPercentage, angleSpeedPercentage), Set.of());
    }

    public static Command getSetCurrentTargetPositionCommand(Rotation2d targetAngle, double targetElevatorPositionMeters, double elevatorSpeedPercentage, double angleSpeedPercentage) {
        if (ARM.isElevatorOpening(targetElevatorPositionMeters)) {
            return new SequentialCommandGroup(
                    getSetTargetAngleCommand(targetAngle, angleSpeedPercentage).until(() -> ARM.atAngle(targetAngle)),
                    getSetTargetElevatorPositionCommand(targetElevatorPositionMeters, elevatorSpeedPercentage)
            );
        }
        return new SequentialCommandGroup(
                getSetTargetElevatorPositionCommand(targetElevatorPositionMeters, elevatorSpeedPercentage).until(() -> ARM.atElevatorMeters(targetElevatorPositionMeters)),
                getSetTargetAngleCommand(targetAngle, angleSpeedPercentage)
        );
    }

    public static Command getSetTargetAngleCommand(Rotation2d targetAngle, double SpeedPercentage) {
        return new FunctionalCommand(
                () -> ARM.generateAngleMotorProfile(targetAngle, SpeedPercentage),
                ARM::setTargetAngleFromProfile,
                (interrupted) -> {
                },
                () -> false,
                ARM
        );
    }

    public static Command getSetTargetElevatorPositionCommand(double targetElevatorPositionMeters, double SpeedPercentage) {
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
