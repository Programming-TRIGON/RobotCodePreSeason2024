package frc.trigon.robot.subsystems.collector;

import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.collector.kablamacollector.KablamaCollectorIO;
import org.littletonrobotics.junction.AutoLog;

public class CollectorIO {
    static CollectorIO generateIO(){
        if (RobotConstants.IS_REPLAY)
            return new CollectorIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.KABLAMA)
            return new KablamaCollectorIO();
        return new SimulationCollectorIO();
    }
    protected void updateInputs(CollectorInputsAutoLogged inputs) {
    }

    protected void setPower(double power) {
    }

    protected void stop() {
    }

    @AutoLog
    protected static class CollectorInputs {
        public double
                motorCurrent = 0,
                motorVoltage = 0;
    }
}
