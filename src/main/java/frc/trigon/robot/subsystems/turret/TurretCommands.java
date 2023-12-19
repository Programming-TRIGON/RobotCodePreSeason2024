package frc.trigon.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import java.util.function.Supplier;

public class TurretCommands {
    private static final Turret TURRET = Turret.getInstance();

    public static Command getSetMotorAngleToHubCommand(Supplier<Pose2d> robotPosition) {
        return new RunCommand(
                () -> TURRET.setMotorAngleToHub(robotPosition.get()),
                TURRET
        );
    }
}