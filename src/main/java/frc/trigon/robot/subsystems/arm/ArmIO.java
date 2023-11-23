package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix6.StatusSignal;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.arm.toohardarm.ToohardArmIO;
import org.littletonrobotics.junction.AutoLog;

public class ArmIO {
    static ArmIO generateIO() {
        if (RobotConstants.IS_REPLAY)
            return new ArmIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.KABLAMA)
            return new ToohardArmIO();
//        return new SimulationArmIO();
        return new ArmIO();
    }

    protected void updateInputs(ArmInputsAutoLogged inputs) {

    }

    protected void setAngleMotorPower(double power) {

    }

    protected void setElevatorMotorPower(double power) {

    }

    protected void stopAngleMotors() {

    }

    protected void stopElevatorMotors() {

    }

    @AutoLog
    protected static class ArmInputs {
        public double angleMotorsVoltage = 0;
        public double elevatorMotorsVoltage = 0;
        public double elevatorPositionRevolutions = 0;
        public double elevatorVelocityRevolutionsPerSecond = 0;
        public StatusSignal<Double> angleEncoderPositionSignal = null;
        public StatusSignal<Double> angleEncoderVelocitySignal = null;
    }
}
