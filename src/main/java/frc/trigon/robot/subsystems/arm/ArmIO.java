package frc.trigon.robot.subsystems.arm;

import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.arm.kablamaArm.KablamaArmIO;
import org.littletonrobotics.junction.AutoLog;

public class ArmIO {

    static ArmIO generateIO() {
        if (RobotConstants.IS_REPLAY)
            return new ArmIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.KABLAMA)
            return new KablamaArmIO();
        //return new SimulationArmIO();
        return new ArmIO();
    }

    protected void updateInputs(ArmInputsAutoLogged inputs) {
    }

    protected void setAnglePower(double power) {
    }

    protected void setElevatorPower(double power) {
    }

    protected void stopAngleMotors() {
    }

    protected void stopElevatorMotors() {
    }

    @AutoLog
    protected static class ArmInputs {
        public double angleMotorCurrent = 0;
        public double angleMotorVoltage = 0;

        public double elevatorMotorCurrent = 0;
        public double elevatorMotorVoltage = 0;
    }
}
