package frc.trigon.robot.subsystems.turret;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

    void alignToHub(Pose2d robotPosition) {
        Rotation2d targetAngle = calculateAngleToHub(robotPosition);
        turretIO.setTargetAngle(limitAngle(targetAngle));
    }

    Rotation2d calculateAngleToHub(Pose2d robotPosition) {
        double yDistance = Math.abs(robotPosition.getY() - TurretConstants.HUB_POSITION.getY());
        double xDistance = Math.abs(robotPosition.getX() - TurretConstants.HUB_POSITION.getX());
        double targetAngle = Math.atan2(yDistance, xDistance);
        return Rotation2d.fromDegrees(targetAngle + robotPosition.getRotation().getDegrees());
    }

    private static Rotation2d limitAngle(Rotation2d targetAngle) {
        if (isOverMaximumAngle(targetAngle)) {
            return targetAngle.minus(Rotation2d.fromDegrees(360));
        } else if (isUnderMinimumAngle(targetAngle)) {
            return targetAngle.plus(Rotation2d.fromDegrees(360));
        } else return targetAngle;
    }

    private static boolean isOverMaximumAngle(Rotation2d targetAngle) {
        return targetAngle.getDegrees() > TurretConstants.ANGLE_MAXIMUM_DEGREES;
    }

    private static boolean isUnderMinimumAngle(Rotation2d targetAngle) {
        return targetAngle.getDegrees() < TurretConstants.ANGLE_MINIMUM_DEGREES;
    }
}

