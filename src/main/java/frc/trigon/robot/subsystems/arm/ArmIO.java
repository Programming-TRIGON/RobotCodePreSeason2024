package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.arm.toohardarm.ToohardArmIO;
import org.littletonrobotics.junction.AutoLog;

public class ArmIO {

    protected void updateInputs(ArmInputsAutoLogged inputs) {

    }

    protected void setAngleMotorPower(double power) {

    }

    protected void setElevatorMotorPower(double power) {

    }

    protected void stopAngleMotor() {

    }

    protected void stopElevatorMotor() {

    }

    @AutoLog
    protected static class ArmInputs {
        public double
                masterAngleMotorVoltage = 0,
                followerAngleMotorVoltage = 0,
                masterElevatorMotorVoltage = 0,
                followerElevatorMotorVoltage = 0;
        public double elevatorPositionRevolutions = 0;
        public double elevatorVelocityRevolutions = 0;
        public PIDController anglePIDController = null;
        public PIDController elevatorPIDController = null;
        public ArmFeedforward angleFeedforward = null;
        public ElevatorFeedforward elevatorFeedforward = null;
        public StatusSignal<Double> angleEncoderPositionSignal = null;
        public StatusSignal<Double> angleEncoderVelocitySignal = null;
    }

    static ArmIO generateIO() {
        if (RobotConstants.IS_REPLAY)
            return new ArmIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.KABLAMA)
            return new ToohardArmIO();
        return new SimulationArmIO();
    }
}
