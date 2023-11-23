package frc.trigon.robot.subsystems.arm.toohardarm;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.StatusSignal;
import com.revrobotics.CANSparkMax;
import frc.trigon.robot.subsystems.arm.ArmIO;
import frc.trigon.robot.subsystems.arm.ArmInputsAutoLogged;

public class ToohardArmIO extends ArmIO {

    private final CANSparkMax
            masterAngleMotor = ToohardArmConstants.MASTER_ANGLE_MOTOR,
            followerAngleMotor = ToohardArmConstants.FOLLOWER_ANGLE_MOTOR,
            masterElevatorMotor = ToohardArmConstants.MASTER_ELEVATOR_MOTOR,
            followerElevatorMotor = ToohardArmConstants.FOLLOWER_ELEVATOR_MOTOR;
    protected final TalonSRX elevatorEncoder = ToohardArmConstants.ELEVATOR_ENCODER;
    protected final StatusSignal<Double> angleEncoderPositionSignal = ToohardArmConstants.ANGLE_ENCODER_POSITION_SIGNAL;
    protected final StatusSignal<Double> angleEncoderVelocitySignal = ToohardArmConstants.ANGLE_ENCODER_VELOCITY_SIGNAL;

    @Override
    protected void updateInputs(ArmInputsAutoLogged inputs) {
        inputs.masterAngleMotorVoltage = masterAngleMotor.getBusVoltage();
        inputs.followerAngleMotorVoltage = followerAngleMotor.getBusVoltage();
        inputs.masterElevatorMotorVoltage = masterElevatorMotor.getBusVoltage();
        inputs.followerElevatorMotorVoltage = followerElevatorMotor.getBusVoltage();
        inputs.elevatorPositionRevolutions = elevatorEncoder.getSelectedSensorPosition();
        inputs.elevatorVelocityRevolutions = elevatorEncoder.getSelectedSensorVelocity();
        inputs.anglePIDController = ToohardArmConstants.ANGLE_PID_CONTROLLER;
        inputs.elevatorPIDController = ToohardArmConstants.ELEVATOR_PID_CONTROLLER;
        inputs.angleFeedforward = ToohardArmConstants.ANGLE_FEEDFORWARD;
        inputs.elevatorFeedforward = ToohardArmConstants.ELEVATOR_FEEDFORWARD;
        inputs.angleEncoderPositionSignal = angleEncoderPositionSignal;
        inputs.angleEncoderVelocitySignal = angleEncoderVelocitySignal;
    }

    @Override
    protected void setAngleMotorPower(double power) {
        masterAngleMotor.set(power);
        followerAngleMotor.set(power);
    }

    @Override
    protected void setElevatorMotorPower(double power) {
        masterElevatorMotor.set(power);
        followerElevatorMotor.set(power);
    }

    @Override
    protected void stopAngleMotor() {
        masterAngleMotor.stopMotor();
        followerAngleMotor.stopMotor();
    }

    @Override
    protected void stopElevatorMotor() {
        masterElevatorMotor.stopMotor();
        followerElevatorMotor.stopMotor();
    }
}
