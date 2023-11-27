package frc.trigon.robot.subsystems.arm.toohardarm;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.StatusSignal;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.subsystems.arm.ArmIO;
import frc.trigon.robot.subsystems.arm.ArmInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;

public class ToohardArmIO extends ArmIO {
    private final CANSparkMax
            masterAngleMotor = ToohardArmConstants.MASTER_ANGLE_MOTOR,
            followerAngleMotor = ToohardArmConstants.FOLLOWER_ANGLE_MOTOR,
            masterElevatorMotor = ToohardArmConstants.MASTER_ELEVATOR_MOTOR,
            followerElevatorMotor = ToohardArmConstants.FOLLOWER_ELEVATOR_MOTOR;
    private final TalonSRX elevatorEncoder = ToohardArmConstants.ELEVATOR_ENCODER;
    private final StatusSignal<Double> angleEncoderPositionSignal = ToohardArmConstants.ANGLE_ENCODER_POSITION_SIGNAL;
    private final StatusSignal<Double> angleEncoderVelocitySignal = ToohardArmConstants.ANGLE_ENCODER_VELOCITY_SIGNAL;

    @Override
    protected void updateInputs(ArmInputsAutoLogged inputs) {
        inputs.angleMotorVoltage = masterAngleMotor.getBusVoltage();
        inputs.elevatorMotorVoltage = masterElevatorMotor.getBusVoltage();

        inputs.elevatorPositionRevolutions = getElevatorPositionRevolutions();
        inputs.elevatorVelocityRevolutionsPerSecond = getElevatorVelocityRevolutionsPerSecond();

        inputs.angleEncoderPositionSignal = getAngleEncoderPositionSignal();
        inputs.angleEncoderVelocitySignal = getAngleEncoderVelocitySignal();
    }

    private double getElevatorPositionRevolutions() {
        return Conversions.magTicksToRevolutions(elevatorEncoder.getSelectedSensorPosition());
    }

    private double getElevatorVelocityRevolutionsPerSecond() {
        return Conversions.perHundredMsToPerSecond(Conversions.magTicksToRevolutions(elevatorEncoder.getSelectedSensorVelocity()));
    }

    private double getAngleEncoderPositionSignal() {
        return angleEncoderPositionSignal.getValue();
    }

    private double getAngleEncoderVelocitySignal() {
        return angleEncoderVelocitySignal.getValue();
    }

    @Override
    protected void setTargetAngle(TrapezoidProfile.State targetState) {
        double pidOutput = ToohardArmConstants.ANGLE_PID_CONTROLLER.calculate(
                angleEncoderPositionSignal.getValue(),
                targetState.position
        );
        double feedforward = ToohardArmConstants.ANGLE_FEEDFORWARD.calculate(
                Units.degreesToRadians(targetState.position),
                targetState.velocity
        );
        masterAngleMotor.set(pidOutput + feedforward);
        followerAngleMotor.set(pidOutput + feedforward);
    }

    @Override
    protected void setTargetElevatorPosition(TrapezoidProfile.State targetState) {
        double pidOutput = ToohardArmConstants.ELEVATOR_PID_CONTROLLER.calculate(
                elevatorEncoder.getSelectedSensorPosition(),
                targetState.position
        );
        double feedforward = ToohardArmConstants.ELEVATOR_FEEDFORWARD.calculate(targetState.velocity);
        masterElevatorMotor.set(pidOutput + feedforward);
        followerElevatorMotor.set(pidOutput + feedforward);
    }

    @Override
    protected void stopAngleMotors() {
        masterAngleMotor.stopMotor();
        followerAngleMotor.stopMotor();
    }

    @Override
    protected void stopElevatorMotors() {
        masterElevatorMotor.stopMotor();
        followerElevatorMotor.stopMotor();
    }
}
