package frc.trigon.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class Turret extends SubsystemBase {
    private final static Turret INSTANCE = new Turret();
    private final TurretIO turretIO = TurretIO.generateIO();
    private final TurretInputsAutoLogged turretInputs = new TurretInputsAutoLogged();
    private final Pose2d hubPose = TurretConstants.HUB_POSE;

    public static Turret getInstance() {
        return INSTANCE;
    }

    private Turret() {
    }

    @Override
    public void periodic() {
        turretIO.updateInputs(turretInputs);
        Logger.processInputs("Turret", turretInputs);
    }

    public Command getSetMotorOutputCommand(Supplier<Pose2d> robotPose) {
        return new FunctionalCommand(
                () -> {
                },
                () -> turretIO.setMotorVoltage(calculateMotorOutput(robotPose.get())),
                (interrupted) -> turretIO.stop(),
                () -> false,
                this
        );
    }

    private double calculateMotorOutput(Pose2d robotPose) {
        double targetAngleRadians = calculateTargetAngle(robotPose);
        double targetAngleAfterLimitCheck = limitCheck(Units.radiansToDegrees(targetAngleRadians));
        double error = calculateError(targetAngleAfterLimitCheck);
        double output = TurretConstants.PID_CONTROLLER.calculate(error);
        return output;
    }

    private double calculateTargetAngle(Pose2d robotPose) {
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotHeading = robotPose.getRotation().getRadians();
        double differenceX = hubPose.getX() - robotX;
        double differenceY = hubPose.getY() - robotY;
        double theta = Math.atan2(differenceY, differenceX);
        double targetAngle = theta - robotHeading;
        return targetAngle;
    }

    private double limitCheck(double targetAngleDegrees) {
        if (targetAngleDegrees > 200) {
            targetAngleDegrees -= 360;
        } else if (targetAngleDegrees < -200) {
            targetAngleDegrees += 360;
        }
        return targetAngleDegrees;
    }

    private double calculateError(double targetAngleDegrees) {
        double error = targetAngleDegrees - turretInputs.motorPositionDegrees;
        return error;
    }

}
