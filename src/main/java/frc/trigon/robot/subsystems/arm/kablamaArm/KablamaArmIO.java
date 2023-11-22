package frc.trigon.robot.subsystems.arm.kablamaArm;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
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
    protected void setAnglePower(double power) {
        masterAngleMotor.setVoltage(power);
        followerAngleMotor.setVoltage(power);
    }

    @Override
    protected void setElevatorPower(double power) {
        masterElevatorMotor.setVoltage(power);
        followerElevatorMotor.setVoltage(power);
    }

    @Override
    protected void stopAngleMotors() {
        masterAngleMotor.stopMotor();
        followerAngleMotor.stopMotor();
    }

    protected void stopElevatorMotors() {
        masterElevatorMotor.stopMotor();
        followerElevatorMotor.stopMotor();
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
