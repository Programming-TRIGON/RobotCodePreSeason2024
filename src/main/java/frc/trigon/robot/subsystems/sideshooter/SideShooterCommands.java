package frc.trigon.robot.subsystems.sideshooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class SideShooterCommands {

    public static SideShooterCommands getInstance() {
        return getInstance();
    }


    public Command getSetTargetShooterAngleCommand(Rotation2d targetAngle) {
        return new FunctionalCommand(
                () -> SideShooter.getInstance().generateAngleMotorProfile(targetAngle),
                () -> getSetTargetShooterAngleCommand(targetAngle),
                (interrupted) -> {},
                () -> false,
        SideShooter.getInstance()
        );
    }
}
