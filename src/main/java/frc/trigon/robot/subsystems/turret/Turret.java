package frc.trigon.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
    private final static Turret INSTANCE = new Turret();
    private final TurretIO turretIO = TurretIO.generateIO();
    private final TurretInputsAutoLogged turretInputs = new TurretInputsAutoLogged();

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

    void setMotorVoltage(Pose2d robotPose) {
        Rotation2d targetAngle = calculateTargetAngle(robotPose);
        Rotation2d targetAngleAfterLimitCheck = angleLimitCorrection(targetAngle.getDegrees());
        Rotation2d error = calculateError(targetAngleAfterLimitCheck.getDegrees());
        turretIO.calculateMotorVoltage(error.getDegrees());
    }

    void stop() {
        turretIO.stop();
    }

    private Rotation2d calculateTargetAngle(Pose2d robotPose) {
        Translation2d difference = TurretConstants.HUB_POSITION.minus(robotPose.getTranslation());
        double theta = Math.atan2(difference.getY(), difference.getX());
        double targetAngle = theta - Units.degreesToRadians(turretInputs.motorPositionDegrees) - robotPose.getRotation().getRadians();
        return Rotation2d.fromRadians(targetAngle);
    }

    private Rotation2d angleLimitCorrection(double targetAngleDegrees) {
        if (targetAngleDegrees > TurretConstants.DEGREES_LIMIT)
            targetAngleDegrees -= 360;
        else if (targetAngleDegrees < -TurretConstants.DEGREES_LIMIT)
            targetAngleDegrees += 360;

        return Rotation2d.fromDegrees(targetAngleDegrees);
    }

    private Rotation2d calculateError(double targetAngleDegrees) {
        return Rotation2d.fromDegrees(targetAngleDegrees - turretInputs.motorPositionDegrees);
    }
}
