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
        if (Math.abs(targetAngle.getDegrees() - robotPosition.getRotation().getDegrees()) > TurretConstants.TOLERANCE_DEGREES) {
            if (isOverMaximumAngle(targetAngle)) {
                turretIO.setTargetAngle(targetAngle.minus(Rotation2d.fromDegrees(360)));
            } else if (isUnderMinimumAngle(targetAngle)) {
                turretIO.setTargetAngle(targetAngle.plus(Rotation2d.fromDegrees(360)));
            } else {
                turretIO.setTargetAngle(targetAngle);
            }
        }
    }

    Rotation2d calculateAngleToHub(Pose2d robotPosition) {
        double
                yDistance = Math.abs(robotPosition.getY() - TurretConstants.HUB_POSITION.getY()),
                xDistance = Math.abs(robotPosition.getX() - TurretConstants.HUB_POSITION.getX()),
                targetAngle = Math.atan2(yDistance, xDistance);
        return Rotation2d.fromRadians(targetAngle + robotPosition.getRotation().getDegrees());
    }

    private static boolean isOverMaximumAngle(Rotation2d targetAngle) {
        return targetAngle.getDegrees() > TurretConstants.ANGLE_MAXIMUM_DEGREES;
    }

    private static boolean isUnderMinimumAngle(Rotation2d targetAngle) {
        return targetAngle.getDegrees() < TurretConstants.ANGLE_MINIMUM_DEGREES;
    }
}

