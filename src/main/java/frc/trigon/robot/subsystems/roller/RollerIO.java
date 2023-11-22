package frc.trigon.robot.subsystems.roller;

import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.roller.toohardroller.ToohardRollerIO;
import org.littletonrobotics.junction.AutoLog;

public class RollerIO {

    protected void updateInputs(RollerInputsAutoLogged inputs) {
    }

    protected void setAngleMotorPower(double power) {
    }

    protected void setCollectionMotorPower(double power) {
    }

    protected void stopAngleMotor() {
    }

    protected void stopCollectionMotor() {
    }

    @AutoLog
    protected static class RollerInputs {
        public double
                angleMotorCurrent = 0,
                angleMotorVoltage = 0,
                collectionMotorCurrent = 0,
                collectionMotorVoltage = 0;
    }
    static RollerIO generateIO()    {
        if (RobotConstants.IS_REPLAY)
            return new RollerIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.KABLAMA)
            return new ToohardRollerIO();
        return new SimulationRollerIO();
    }
}
