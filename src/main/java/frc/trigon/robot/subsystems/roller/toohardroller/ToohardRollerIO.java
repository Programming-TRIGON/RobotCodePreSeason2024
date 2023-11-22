package frc.trigon.robot.subsystems.roller.toohardroller;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import frc.trigon.robot.subsystems.roller.RollerIO;
import frc.trigon.robot.subsystems.roller.RollerInputsAutoLogged;

public class ToohardRollerIO extends RollerIO {
    private final TalonSRX angleMotor = ToohardRollerConstants.ANGLE_MOTOR;
    private final CANSparkMax collectionMotor = ToohardRollerConstants.COLLECTION_MOTOR;
    @Override
    protected void updateInputs(RollerInputsAutoLogged inputs) {
        inputs.angleMotorCurrent = angleMotor.getSupplyCurrent();
        inputs.angleMotorVoltage = angleMotor.getMotorOutputVoltage();
        inputs.collectionMotorCurrent = collectionMotor.getOutputCurrent();
        inputs.collectionMotorVoltage = collectionMotor.getBusVoltage();
    }

    @Override
    protected void setAngleMotorPower(double power) {
        angleMotor.set(ControlMode.PercentOutput, power);
    }

    @Override
    protected void stopAngleMotor() {
        angleMotor.set(ControlMode.Disabled, 0);
    }

    @Override
    protected void setCollectionMotorPower(double power) {
        collectionMotor.set(power);
    }

    @Override
    protected void stopCollectionMotor() {
        collectionMotor.set(0);
    }
}
