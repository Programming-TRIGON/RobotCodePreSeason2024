package frc.trigon.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

import java.util.function.Supplier;

public class TurretCommands {
    private static final Turret TURRET = Turret.getInstance();

    private static Command getSetTargetAngleCommand(Supplier<Pose2d> robotPosition, double speedPercentage) {
        return new FunctionalCommand(
                () -> TURRET.generateMotorProfile(TURRET.calculateAngleToHub(robotPosition.get()), speedPercentage),
                TURRET::setTargetAngleFromProfile,
                (interrupted) -> {
                },
                () -> false,
                TURRET
        );
    }
}