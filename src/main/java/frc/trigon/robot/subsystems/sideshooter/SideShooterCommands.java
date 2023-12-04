package frc.trigon.robot.subsystems.sideshooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;

public class SideShooterCommands {
    public Command getSetTargetStateCommand(Rotation2d targetAngle, double targetVoltage, boolean byOrder) {
        if (!byOrder) {
            return new ParallelCommandGroup(
                    getSetTargetShooterAngleCommand(targetAngle),
                    getSetTargetShootingVoltageCommand(targetVoltage)
            );
        }
        return new SequentialCommandGroup(
                getSetTargetShooterAngleCommand(targetAngle),
                getSetTargetShootingVoltageCommand(targetVoltage)
        );
    }

    public static Command getSetTargetShooterAngleCommand(Rotation2d targetAngle) {
        return new FunctionalCommand(
                () -> SideShooter.getInstance().generateAngleMotorProfile(targetAngle),
                () -> SideShooter.getInstance().setTargetAngleFromProfile(),
                (interrupted) -> {
                },
                () -> false,
                SideShooter.getInstance()
        );
    }

    public Command getSetTargetShootingVoltageCommand(double targetVoltage) {
        return new StartEndCommand(
                () -> SideShooter.getInstance().setTargetShootingVoltage(targetVoltage),
                () -> SideShooter.getInstance().stopAngleMotor(),
                SideShooter.getInstance()
        );
    }
}
