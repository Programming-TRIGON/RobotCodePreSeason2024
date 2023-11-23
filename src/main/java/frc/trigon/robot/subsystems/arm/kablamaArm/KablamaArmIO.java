package frc.trigon.robot.subsystems.arm.kablamaArm;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.subsystems.arm.ArmIO;
import frc.trigon.robot.subsystems.arm.ArmInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;

public class KablamaArmIO extends ArmIO {
    private final CANSparkMax
            masterAngleMotor = KablamaArmConstants.MASTER_ANGLE_MOTOR,
            followerAngleMotor = KablamaArmConstants.FOLLOWER_ANGLE_MOTOR,
            masterElevatorMotor = KablamaArmConstants.MASTER_ELEVATOR_MOTOR,
            followerElevatorMotor = KablamaArmConstants.FOLLOWER_ELEVATOR_MOTOR;

    @Override
    protected void updateInputs(ArmInputsAutoLogged inputs) {
        inputs.angleMotorVoltage = masterAngleMotor.getBusVoltage();
        inputs.angleMotorCurrent = masterAngleMotor.getOutputCurrent();
        inputs.elevatorMotorVoltage = masterElevatorMotor.getBusVoltage();
        inputs.elevatorMotorCurrent = masterElevatorMotor.getOutputCurrent();
        inputs.elevatorPositionRevolution = getElevatorPositionRevolutions();
        inputs.angleVelocityDegreesPerSecond = getAngleVelocityDegreesPerSecond();
        inputs.elevatorVelocityRevolutionsPerSecond = getElevatorVelocityRevolutionsPerSecond();
        inputs.anglePositionDegrees = getAngleMotorPositionDegrees().getDegrees();
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

    @Override
    protected void setTargetAngleState(TrapezoidProfile.State targetState) {
        double pidOutput = KablamaArmConstants.ANGLE_PID_CONTROLLER.calculate(
                getAngleMotorPositionDegrees().getDegrees(),
                targetState.position
        );
        double feedforward = KablamaArmConstants.ANGLE_FEEDFORWARD.calculate(
                Units.degreesToRadians(targetState.position),
                Units.degreesToRadians(targetState.velocity)
        );
        masterAngleMotor.setVoltage(pidOutput + feedforward);
        followerAngleMotor.setVoltage(pidOutput + feedforward);
    }

    @Override
    protected void setTargetElevatorPositionState(TrapezoidProfile.State targetState) {
        double pidOutput = KablamaArmConstants.ELEVATOR_PID_CONTROLLER.calculate(
                getElevatorPositionRevolutions(),
                targetState.position
        );
        double feedforward = KablamaArmConstants.ELEVATOR_FEEDFORWARD.calculate(targetState.velocity);

        masterElevatorMotor.setVoltage(pidOutput + feedforward);
        followerElevatorMotor.setVoltage(pidOutput + feedforward);
    }

    protected double getElevatorPositionRevolutions() {
        return Conversions.magTicksToRevolutions(KablamaArmConstants.ELEVATOR_ENCODER.getSelectedSensorPosition());
    }

    protected double getAngleVelocityDegreesPerSecond() {
        double positionRevolutions = KablamaArmConstants.ANGLE_MOTOR_VELOCITY_SIGNAL.refresh().getValue();
        return Conversions.revolutionsToDegrees(positionRevolutions);
    }

    protected double getElevatorVelocityRevolutionsPerSecond() {
        double magTicksPerSecond = Conversions.perHundredMsToPerSecond(KablamaArmConstants.ELEVATOR_ENCODER.getSelectedSensorVelocity());
        return Conversions.magTicksToRevolutions(magTicksPerSecond);
    }

    protected Rotation2d getAngleMotorPositionDegrees() {
        double positionRevolutions = KablamaArmConstants.ANGLE_MOTOR_POSITION_SIGNAL.refresh().getValue();
        return Rotation2d.fromRotations(positionRevolutions);
    }
}
