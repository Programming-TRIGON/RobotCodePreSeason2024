package frc.trigon.robot.subsystems.turret;

import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.turret.simulationturret.SimulationTurretIO;
import frc.trigon.robot.subsystems.turret.toohardturret.ToohardTurretIO;
import org.littletonrobotics.junction.AutoLog;

public class TurretIO {
    static TurretIO generateIO() {
        if (RobotConstants.IS_REPLAY)
            return new TurretIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.KABLAMA)
            return new ToohardTurretIO();
        return new SimulationTurretIO();
    }

    protected void updateInputs(TurretInputsAutoLogged inputs) {
    }

    protected void setTargetAnglePower(double power) {
    }

    protected void stopMotor() {
    }

    @AutoLog
    protected static class TurretInputs {
        public double motorVoltage = 0;
        public double motorAngleDegrees = 0;
        public double motorVelocityDegreesPerSecond = 0;
    }
}
