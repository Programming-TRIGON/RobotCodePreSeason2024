package frc.trigon.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.turret.kablamaturret.KablamaTurretIO;
import org.littletonrobotics.junction.AutoLog;

public class TurretIO {

    static TurretIO generateIO() {
        if (RobotConstants.IS_REPLAY)
            return new TurretIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.KABLAMA)
            return new KablamaTurretIO();
        // return new SimulationTurretIO;
        return new TurretIO();
    }

    protected void updateInputs(TurretInputsAutoLogged inputs) {
    }

    protected void calculateMotorOutput(Pose2d robotPose) {
    }

    protected void stop() {
    }

    @AutoLog

    protected static class TurretInputs {
        public double motorPositionDegrees = 0;
    }
}
