package frc.trigon.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ArmCommands {
    private static final Arm ARM = Arm.getInstance();

    public static Command getSetTargetStateCommand(ArmConstants.ArmState state){
        return getSetTargetPositionCommand(state.angle, state.elevatorPositionMeters, 100, 100);
    }

    public static Command getsetTargetStateCommand(ArmConstants.ArmState state, double elevatorSpeedPercentages, double angleSpeedPercentages){
        return getSetTargetPositionCommand(state.angle, state.elevatorPositionMeters,elevatorSpeedPercentages, angleSpeedPercentages);
    }

    public static Command getSetTargetPositionCommand(Rotation2d targetAngle, double targetElevatorPositionMeters, double elevatorSpeedPercentages, double angleSpeedPercentage) {
        if (ARM.isElevatorOpening(targetElevatorPositionMeters)) {
            return new SequentialCommandGroup(
                    getSetTargetAngleCommand(targetAngle, angleSpeedPercentage).until(() -> ARM.atAngle(targetAngle)),
                    getSetTargetElevatorPositionMetersCommand(targetElevatorPositionMeters,elevatorSpeedPercentages)
            );
        }
        return new SequentialCommandGroup(
                getSetTargetElevatorPositionMetersCommand(targetElevatorPositionMeters, elevatorSpeedPercentages).until(()-> ARM.atElevatorMeters(targetElevatorPositionMeters)),
                getSetTargetAngleCommand(targetAngle, angleSpeedPercentage)
        );
    }

    public static Command getSetTargetAngleCommand(Rotation2d targetAngle, double angleSpeedPercentage) {
        return new FunctionalCommand(
                () -> ARM.generateAngleMotorProfile(targetAngle, angleSpeedPercentage),
                ARM::setTargetAngleFromProfile,
                (interrupted) -> {
                },
                () -> false,
                ARM
        );
    }

    public static Command getSetTargetElevatorPositionMetersCommand(double targetElevatorPositionMeters, double elevatorSpeedPercentage) {
        return new FunctionalCommand(
                () -> ARM.generateElevatorMotorProfile(targetElevatorPositionMeters, elevatorSpeedPercentage),
                ARM::setTargetElevatorPositionFromProfile,
                (interrupted) -> {
                },
                () -> false,
                ARM
        );
    }
}
