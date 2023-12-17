package frc.trigon.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

import java.util.function.Supplier;

public class TurretCommands {
    private static Turret TURRET = Turret.getInstance();

    public static Command getSetMotorVoltageCommand(Supplier<Pose2d> robotPose) {
        return new FunctionalCommand(
                () -> {
                },
                () -> TURRET.setMotorVoltage(robotPose.get()),
                (interrupted) -> TURRET.stop(),
                () -> false,
                TURRET
        );
    }
}
