package frc.trigon.robot.subsystems.sideshooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.utilities.Commands;

public class SideShooterCommands {
    private static final SideShooter SIDE_SHOOTER = SideShooter.getInstance();

    public static Command getSetTargetStateCommand(Rotation2d targetAngle, double targetVoltage, boolean byOrder) {
        if (!byOrder) {
            return new ParallelCommandGroup(
                    Commands.removeRequirements(getSetTargetShooterAngleCommand(targetAngle)),
                    getSetTargetShootingVoltageCommand(targetVoltage)
            );
        }
        return new SequentialCommandGroup(
                getSetTargetShooterAngleCommand(targetAngle).until(()-> SIDE_SHOOTER.atAngle(Rotation2d.fromRadians(targetAngle.getDegrees()))),
                getSetTargetShootingVoltageCommand(targetVoltage)
        );
    }

    public static Command getSetTargetShooterAngleCommand(Rotation2d targetAngle) {
        return new FunctionalCommand(
                () -> SIDE_SHOOTER.generateAngleMotorProfile(targetAngle),
                () -> SIDE_SHOOTER.setTargetAngleFromProfile(),
                (interrupted) -> {
                },
                () -> false,
                SIDE_SHOOTER
        );
    }

    public static Command getSetTargetShootingVoltageCommand(double targetVoltage) {
        return new StartEndCommand(
                () -> SIDE_SHOOTER.setTargetShootingVoltage(targetVoltage),
                () -> SIDE_SHOOTER.stopAngleMotor(),
                SIDE_SHOOTER
        );
    }
}
