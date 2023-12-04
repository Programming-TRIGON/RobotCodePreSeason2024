package frc.trigon.robot.subsystems.sideshooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.utilities.Commands;

public class SideShooterCommands {
    private static final SideShooter SIDE_SHOOTER = SideShooter.getInstance();

    public static Command getSetTargetStateCommand(boolean byOrder, SideShooterConstants.SideShooterState targetState) {
        if (!byOrder) {
            return new ParallelCommandGroup(
                    Commands.removeRequirements(getSetTargetAngleCommand(targetState.angle)),
                    getSetTargetShootingVoltageCommand(targetState.voltage)
            );
        }
        return new SequentialCommandGroup(
                Commands.removeRequirements(getSetTargetAngleCommand(targetState.angle).until(() -> SIDE_SHOOTER.atAngle(targetState.angle))),
                getSetTargetShootingVoltageCommand(targetState.voltage)
        );
    }

    public static Command getSetTargetAngleCommand(Rotation2d targetAngle) {
        return new FunctionalCommand(
                () -> SIDE_SHOOTER.generateAngleMotorProfile(targetAngle),
                SIDE_SHOOTER::setTargetAngleFromProfile,
                (interrupted) -> {
                },
                () -> false,
                SIDE_SHOOTER
        );
    }

    public static Command getSetTargetShootingVoltageCommand(double targetVoltage) {
        return new StartEndCommand(
                () -> SIDE_SHOOTER.setTargetShootingVoltage(targetVoltage),
                SIDE_SHOOTER::stopShooting,
                SIDE_SHOOTER
        );
    }
}