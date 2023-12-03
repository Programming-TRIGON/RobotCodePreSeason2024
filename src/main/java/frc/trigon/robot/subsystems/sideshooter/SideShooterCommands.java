package frc.trigon.robot.subsystems.sideshooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;

public class SideShooterCommands {
    private static final SideShooter SIDE_SHOOTER = SideShooter.getInstance();

    public static Command getSetTargetStateCommand(boolean byOrder, SideShooterConstants.SideShooterState targetState) {
        if (!byOrder) {
            return new ParallelCommandGroup(
                    getSetTargetAngleCommand(targetState.angle),
                    getSetTargetShootingVoltageCommand(targetState.voltage)
            );
        }
        return new SequentialCommandGroup(
                getSetTargetAngleCommand(targetState.angle).until(()-> SIDE_SHOOTER.atAngle(targetState.angle)),
                getSetTargetShootingVoltageCommand(targetState.voltage)
        );
    }

    public static Command getSetTargetAngleCommand(Rotation2d targetAngle) {
        return new FunctionalCommand(
                () -> SideShooter.getInstance().generateAngleMotorProfile(targetAngle),
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