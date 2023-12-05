package frc.trigon.robot.subsystems.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.arm.simulationarm.SimulationArmIO;
import frc.trigon.robot.subsystems.arm.toohardarm.ToohardArmIO;
import org.littletonrobotics.junction.AutoLog;

public class ArmIO {
    static ArmIO generateIO() {
        if (RobotConstants.IS_REPLAY)
            return new ArmIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.KABLAMA)
            return new ToohardArmIO();
        return new SimulationArmIO();
    }

    protected void updateInputs(ArmInputsAutoLogged inputs) {
    }

    protected void setTargetAngleState(TrapezoidProfile.State targetState) {
    }

    protected void setTargetElevatorState(TrapezoidProfile.State targetState) {
    }

    protected void stopAngleMotors() {
    }

    protected void stopElevatorMotors() {
    }

    @AutoLog
    protected static class ArmInputs {
        public double elevatorMotorVoltage = 0;
        public double elevatorPositionRevolutions = 0;
        public double elevatorVelocityRevolutionsPerSecond = 0;

        public double angleMotorVoltage = 0;
        public double anglePositionDegrees = 0;
        public double angleVelocityDegreesPerSecond = 0;
    }
}
