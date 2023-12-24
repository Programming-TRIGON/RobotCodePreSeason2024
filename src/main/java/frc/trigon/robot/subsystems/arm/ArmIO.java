package frc.trigon.robot.subsystems.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.arm.kablamaarm.KablamaArmIO;
import frc.trigon.robot.subsystems.arm.simulationarm.SimulationArmIO;
import org.littletonrobotics.junction.AutoLog;

public class ArmIO {
    static ArmIO generateIO() {
        if (RobotConstants.IS_REPLAY)
            return new ArmIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.KABLAMA)
            return new KablamaArmIO();
        return new SimulationArmIO();
    }

    protected void updateInputs(ArmInputsAutoLogged inputs) {
    }

    protected void stopAngleMotors() {
    }

    protected void stopElevatorMotors() {
    }

    protected void setTargetAngleState(TrapezoidProfile.State targetState) {
    }

    protected void setTargetElevatorState(TrapezoidProfile.State targetState) {
    }

    @AutoLog
    protected static class ArmInputs {
        public double angleMotorCurrent = 0;
        public double angleMotorVoltage = 0;
        public double anglePositionDegrees = 0;
        public double angleVelocityDegreesPerSecond = 0;

        public double elevatorMotorCurrent = 0;
        public double elevatorMotorVoltage = 0;
        public double elevatorPositionMeters = 0;
        public double elevatorVelocityMetersPerSecond = 0;
    }
}
