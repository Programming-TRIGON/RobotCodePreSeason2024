package frc.trigon.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import java.util.function.Supplier;

public class TurretCommands {
    private static final Turret TURRET = Turret.getInstance();

    public static Command getAlignTurretToHubCommand(Supplier<Pose2d> robotPose) {
        return new RunCommand(
                () -> TURRET.alignTurretToHub(robotPose.get()),
                TURRET
        );
    }
}
