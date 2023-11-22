package frc.trigon.robot.subsystems.arm.kablamaArm;

import com.revrobotics.CANSparkMax;
import frc.trigon.robot.subsystems.arm.ArmConstants;
import frc.trigon.robot.subsystems.arm.ArmIO;
import frc.trigon.robot.subsystems.arm.ArmInputsAutoLogged;

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
    }

    @Override
    protected void setAnglePower(double power) {
        masterAngleMotor.setVoltage(power);
        followerAngleMotor.setVoltage(power);
    }

    @Override
    protected void setElevatorPower(double power) {
        masterAngleMotor.setVoltage(power);
        followerAngleMotor.setVoltage(power);
    }

    @Override
    protected void stopAngleMotors() {
        masterAngleMotor.stopMotor();
        followerAngleMotor.stopMotor();
    }

    @Override
    protected void stopElevatorMotors() {
        masterElevatorMotor.stopMotor();
        followerAngleMotor.stopMotor();
    }
}
