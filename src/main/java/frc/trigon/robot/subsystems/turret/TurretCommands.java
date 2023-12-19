package frc.trigon.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class TurretCommands {
    private static final Turret TURRET = Turret.getInstance();

    public static Command getSetMotorPowerFromPositionCommand(Pose2d robotPosition) {
        return new InstantCommand(
                () -> TURRET.setMotorPowerFromPosition(robotPosition),
                TURRET
        );
    }
}