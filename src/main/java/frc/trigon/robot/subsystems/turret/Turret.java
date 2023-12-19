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

    /**
     * @param robotPosition position of the robot on the field
     * @return a command that sets the turret to look at the centre hub
     */
    void setMotorPowerFromPosition(Pose2d robotPosition) {
        Rotation2d targetAngle = calculateAngleToHub(robotPosition);
        Rotation2d robotAngle = robotPosition.getRotation();
        if (Math.abs(targetAngle.getDegrees() - robotAngle.getCos()) > TurretConstants.TOLERANCE) {
            if (Math.abs(targetAngle.getDegrees() + robotAngle.getDegrees()) < 200) {
                turretIO.setTargetAnglePower(1);
            } else if (Math.abs(targetAngle.getDegrees() + robotAngle.getDegrees()) > 200) {
                turretIO.setTargetAnglePower(-1);
            }
        }
    }

    Rotation2d calculateAngleToHub(Pose2d robotPosition) {
        double
                yDistance = Math.abs(robotPosition.getY() - TurretConstants.HUB_POSITION.getY()),
                xDistance = Math.abs(robotPosition.getX() - TurretConstants.HUB_POSITION.getX());
        return Rotation2d.fromDegrees(Math.atan(xDistance / yDistance));
    }
}

