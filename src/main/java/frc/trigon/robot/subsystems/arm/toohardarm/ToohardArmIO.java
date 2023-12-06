package frc.trigon.robot.subsystems.arm.toohardarm;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
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

    @Override
    protected void updateInputs(ArmInputsAutoLogged inputs) {
        inputs.elevatorMotorVoltage = masterElevatorMotor.getBusVoltage();
        inputs.elevatorPositionMeters = getElevatorPositionMeters();
        inputs.elevatorVelocityMetersPerSecond = getElevatorVelocityMetersPerSecond();

        inputs.angleMotorVoltage = masterAngleMotor.getBusVoltage();
        inputs.anglePositionDegrees = getAnglePositionDegrees();
        inputs.angleVelocityDegreesPerSecond = getAngleVelocityDegreesPerSecond();
    }

    @Override
    protected void setTargetAngleState(TrapezoidProfile.State targetState) {
        setAngleMotorsVoltage(calculateAngleVoltageFromState(targetState));
    }

    @Override
    protected void setTargetElevatorState(TrapezoidProfile.State targetState) {
        setElevatorMotorsVoltage(calculateElevatorVoltageFromState(targetState));
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

    private double calculateElevatorVoltageFromState(TrapezoidProfile.State targetState) {
        double pidOutput = ToohardArmConstants.ELEVATOR_PID_CONTROLLER.calculate(
                getElevatorPositionMeters(),
                targetState.position
        ) * ToohardArmConstants.VOLTAGE_COMPENSATION_SATURATION;
        double feedforward = ToohardArmConstants.ELEVATOR_FEEDFORWARD.calculate(targetState.velocity) * ToohardArmConstants.VOLTAGE_COMPENSATION_SATURATION;
        return pidOutput + feedforward;
    }

    private double calculateAngleVoltageFromState(TrapezoidProfile.State targetState) {
        double pidOutput = ToohardArmConstants.ANGLE_PID_CONTROLLER.calculate(
                getAnglePositionDegrees(),
                targetState.position
        ) * ToohardArmConstants.VOLTAGE_COMPENSATION_SATURATION;
        double feedforward = ToohardArmConstants.ANGLE_FEEDFORWARD.calculate(
                Units.degreesToRadians(targetState.position),
                targetState.velocity
        ) * ToohardArmConstants.VOLTAGE_COMPENSATION_SATURATION;
        return pidOutput + feedforward;
    }

    private double getElevatorPositionMeters() {
        return Conversions.magTicksToRevolutions(elevatorEncoder.getSelectedSensorPosition()) * ToohardArmConstants.ELEVATOR_METERS_PER_REVOLUTION;
    }

    private double getElevatorVelocityMetersPerSecond() {
        return Conversions.magTicksToRevolutions(elevatorEncoder.getSelectedSensorVelocity()) * ToohardArmConstants.ELEVATOR_METERS_PER_REVOLUTION;
    }

    private double getAnglePositionDegrees() {
        return Conversions.revolutionsToDegrees(ToohardArmConstants.ANGLE_ENCODER_POSITION_SIGNAL.refresh().getValue());
    }

    private double getAngleVelocityDegreesPerSecond() {
        return Conversions.revolutionsToDegrees(ToohardArmConstants.ANGLE_ENCODER_VELOCITY_SIGNAL.refresh().getValue());
    }

    private void setAngleMotorsVoltage(double voltage) {
        masterAngleMotor.setVoltage(voltage);
        followerAngleMotor.setVoltage(voltage);
    }

    private void setElevatorMotorsVoltage(double voltage) {
        masterElevatorMotor.setVoltage(voltage);
        followerElevatorMotor.setVoltage(voltage);
    }
}
