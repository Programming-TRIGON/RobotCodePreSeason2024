package frc.trigon.robot.subsystems.sideshooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.utilities.Commands;

public class SideShooterCommands {

    private static final SideShooter sideShooter = SideShooter.getInstance();
    public static Command getSetTargetStateCommand(Rotation2d targetAngle, double targetVoltage, boolean byOrder) {
        if (!byOrder) {
            return new ParallelCommandGroup(
                    Commands.removeRequirements(getSetTargetShooterAngleCommand(targetAngle)),
                    getSetTargetShootingVoltageCommand(targetVoltage)
            );
        }
        return new SequentialCommandGroup(
                getSetTargetShooterAngleCommand(targetAngle).until(()->sideShooter.ifGotToAngle(targetAngle.getDegrees())),
                getSetTargetShootingVoltageCommand(targetVoltage)
        );
    }

    public static Command getSetTargetShooterAngleCommand(Rotation2d targetAngle) {
        return new FunctionalCommand(
                () -> sideShooter.generateAngleMotorProfile(targetAngle),
                () -> sideShooter.setTargetAngleFromProfile(),
                (interrupted) -> {
                },
                () -> false,
                sideShooter
        );
    }

    public static Command getSetTargetShootingVoltageCommand(double targetVoltage) {
        return new StartEndCommand(
                () -> sideShooter.setTargetShootingVoltage(targetVoltage),
                () -> sideShooter.stopAngleMotor(),
                sideShooter
        );
    }
}
