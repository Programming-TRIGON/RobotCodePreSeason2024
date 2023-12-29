package frc.trigon.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;

import java.util.Set;

public class ArmCommands {
    private static final Arm ARM = Arm.getInstance();

    public Command getSetTargetArmStateCommand(ArmConstants.ArmState targetState) {
        return getSetTargetArmStateCommand(targetState.elevatorPositionMeters, targetState.angle, 100, 100);
    }

    public Command getSetTargetArmStateCommand(ArmConstants.ArmState targetState, double angleSpeedPercentage, double elevatorSpeedPercentage) {
        return getSetTargetArmStateCommand(targetState.elevatorPositionMeters, targetState.angle, angleSpeedPercentage, elevatorSpeedPercentage);
    }

    public Command getSetTargetArmStateCommand(double elevatorPosition, Rotation2d angle, double angleSpeedPercentage, double elevatorSpeedPercentage) {
        return new DeferredCommand(
                () -> ARM.getCurrentSetTargetStateCommand(elevatorPosition, angle, angleSpeedPercentage, elevatorSpeedPercentage),
                Set.of(ARM)
        );
    }

    public Command getSetTargetArmPositionCommand(double elevatorPosition, Rotation2d angle) {
        return new DeferredCommand(
                () -> ARM.getCurrentSetTargetStateCommand(elevatorPosition, angle, 100, 100),
                Set.of(ARM)
        );
    }
}
