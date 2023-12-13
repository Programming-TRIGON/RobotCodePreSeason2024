package frc.trigon.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
    private final static Turret INSTANCE = new Turret();
    private final TurretIO turretIO = TurretIO.generateIO();
    private final TurretInputsAutoLogged turretInputs = new TurretInputsAutoLogged();
    private final Translation2d hubPosition = TurretConstants.HUB_POSITION;

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

    public void calculateMotorVoltage(Pose2d robotPose) {
        double targetAngleRadians = calculateTargetAngle(robotPose);
        double targetAngleAfterLimitCheck = spinLimitCheck(Units.radiansToDegrees(targetAngleRadians));
        double error = calculateError(targetAngleAfterLimitCheck);
        turretIO.calculateMotorVoltage(error);
    }

    private double calculateTargetAngle(Pose2d robotPose) {
        Translation2d difference = hubPosition.minus(robotPose.getTranslation());
        double theta = Math.atan2(difference.getY(), difference.getX());
        double targetAngle = theta - Units.degreesToRadians(turretInputs.motorPositionDegrees);
        return targetAngle;
    }

    private double spinLimitCheck(double targetAngleDegrees) {
        if (targetAngleDegrees > TurretConstants.DEGREES_LIMIT) {
            targetAngleDegrees -= 360;
        } else if (targetAngleDegrees < -TurretConstants.DEGREES_LIMIT) {
            targetAngleDegrees += 360;
        }
        return targetAngleDegrees;
    }

    private double calculateError(double targetAngleDegrees) {
        double error = targetAngleDegrees - turretInputs.motorPositionDegrees;
        return error;
    }

    void stop() {
        turretIO.stop();
    }

}
