package frc.trigon.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class ArmCommands {
    private static final Arm ARM = Arm.getInstance();

    public static Command getSetTargetArmAngleCommand(Rotation2d targetAngle) {
        return new FunctionalCommand(
                () -> ARM.generateAngleMotorProfile(targetAngle),
                () -> ARM.setTargetAngleFromProfile(),
                (interrupted) -> {
                },
                () -> false,
                ARM
        );
    }

    public static Command getSetTargetArmElevatorCommand(Rotation2d targetAngle) {
        return new FunctionalCommand(
                () -> ARM.generateAngleMotorProfile(targetAngle),
                () -> ARM.setTargetAngleFromProfile(),
                (interrupted) -> {
                },
                () -> false,
                ARM
        );
    }
}
