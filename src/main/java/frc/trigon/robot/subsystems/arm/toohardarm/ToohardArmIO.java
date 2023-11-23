package frc.trigon.robot.subsystems.arm.toohardarm;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.StatusSignal;
import com.revrobotics.CANSparkMax;
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
        inputs.angleMotorsVoltage = masterAngleMotor.getBusVoltage();
        inputs.elevatorMotorsVoltage = masterElevatorMotor.getBusVoltage();

        inputs.elevatorPositionRevolutions = elevatorEncoder.getSelectedSensorPosition();
        inputs.elevatorVelocityRevolutionsPerSecond = Conversions.perHundredMsToPerSecond(elevatorEncoder.getSelectedSensorVelocity());

        inputs.anglePIDController = ToohardArmConstants.ANGLE_PID_CONTROLLER;
        inputs.elevatorPIDController = ToohardArmConstants.ELEVATOR_PID_CONTROLLER;

        inputs.angleFeedforward = ToohardArmConstants.ANGLE_FEEDFORWARD;
        inputs.elevatorFeedforward = ToohardArmConstants.ELEVATOR_FEEDFORWARD;

        inputs.angleEncoderPositionSignal = ToohardArmConstants.ANGLE_ENCODER_POSITION_SIGNAL;
        inputs.angleEncoderVelocitySignal = ToohardArmConstants.ANGLE_ENCODER_VELOCITY_SIGNAL;
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
