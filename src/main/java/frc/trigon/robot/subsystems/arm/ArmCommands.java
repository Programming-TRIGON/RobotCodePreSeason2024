package frc.trigon.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ArmCommands {
    private static final Arm ARM = Arm.getInstance();

    public static Command getSetTargetStateCommand(Rotation2d targetAngle, double targetElevatorMeters) {
        if (ARM.isElevatorOpening(targetElevatorMeters)) {
            return new SequentialCommandGroup(
                    getSetTargetAngleCommand(targetAngle).until(() -> ARM.atAngle(targetAngle)),
                    getSetTargetElevatorPositionCommand(targetElevatorMeters)
            );
        }
        return new SequentialCommandGroup(
                getSetTargetElevatorPositionCommand(targetElevatorMeters).until(()-> ARM.atElevatorTargetMeters(targetElevatorMeters)),
                getSetTargetAngleCommand(targetAngle)
        );
    }

    public static Command getSetTargetAngleCommand(Rotation2d targetAngle) {
        return new FunctionalCommand(
                () -> ARM.generateAngleMotorProfile(targetAngle),
                ARM::setTargetAngleFromProfile,
                (interrupted) -> {
                },
                () -> false,
                ARM
        );
    }

    public static Command getSetTargetElevatorPositionCommand(double targetElevatorMeters) {
        return new FunctionalCommand(
                () -> ARM.generateElevatorMotorProfile(targetElevatorMeters),
                ARM::setTargetElevatorPositionFromProfile,
                (interrupted) -> {
                },
                () -> false,
                ARM
        );
    }
}
