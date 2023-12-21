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
        updateMechanism();
    }

    void alignTurretToHub(Pose2d robotPose) {
        Rotation2d targetAngle = calculateTargetAngle(robotPose);
        Rotation2d targetAngleAfterLimitCheck = limitAngle(targetAngle);
        turretIO.setTargetAngle(calculateError(targetAngleAfterLimitCheck.getDegrees()));
    }

    void stop() {
        turretIO.stop();
    }

    private Rotation2d calculateTargetAngle(Pose2d robotPose) {
        Translation2d difference = TurretConstants.HUB_POSITION.minus(robotPose.getTranslation());
        double theta = Math.atan2(Math.abs(difference.getY()), Math.abs(difference.getX()));
        double targetAngle = theta - Units.degreesToRadians(turretInputs.motorPositionDegrees);
        return Rotation2d.fromRadians(targetAngle);
    }

    private Rotation2d limitAngle(Rotation2d targetAngle) {
        if (targetAngle.getDegrees() > TurretConstants.DEGREES_LIMIT)
            return Rotation2d.fromDegrees(targetAngle.getDegrees() - 360);
        if (targetAngle.getDegrees() < -TurretConstants.DEGREES_LIMIT)
            return Rotation2d.fromDegrees(targetAngle.getDegrees() + 360);
        return targetAngle;
    }

    private Rotation2d calculateError(double targetAngleDegrees) {
        return Rotation2d.fromDegrees(targetAngleDegrees - turretInputs.motorPositionDegrees);
    }

    private void updateMechanism() {
        TurretConstants.TURRET_LIGAMENT.setAngle(turretInputs.motorPositionDegrees);
        Logger.recordOutput("TurretMechanism", TurretConstants.TURRET_MECHANISM);
    }
}
