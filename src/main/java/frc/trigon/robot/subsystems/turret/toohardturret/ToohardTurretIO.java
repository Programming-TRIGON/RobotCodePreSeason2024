package frc.trigon.robot.subsystems.turret.toohardturret;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import frc.trigon.robot.subsystems.turret.TurretIO;
import frc.trigon.robot.subsystems.turret.TurretInputsAutoLogged;

public class ToohardTurretIO extends TurretIO {
    private final TalonFX motor = ToohardTurretConstants.MOTOR;
    private final PositionVoltage positionRequest = new PositionVoltage(
            0,
            0,
            ToohardTurretConstants.FOC_ENABLED,
            0,
            0,
            false
    );

    @Override
    protected void updateInputs(TurretInputsAutoLogged inputs) {
        inputs.motorVoltage = motor.getMotorVoltage().refresh().getValue();
        inputs.motorAngleDegrees = ToohardTurretConstants.ENCODER_POSITION_SIGNAL.refresh().getValue();
        inputs.motorVelocityDegreesPerSecond = ToohardTurretConstants.ENCODER_VELOCITY_SIGNAL.refresh().getValue();
    }

    @Override
    protected void setTargetAnglePower(double power) {
        setMotorVoltageFromPower(power);
    }

    @Override
    protected void stopMotor() {
        motor.stopMotor();
    }

    private void setMotorVoltageFromPower(double power) {
        double voltage = MathUtil.clamp(
                power * ToohardTurretConstants.VOLTAGE_COMPENSATION_SATURATION,
                -ToohardTurretConstants.VOLTAGE_COMPENSATION_SATURATION,
                ToohardTurretConstants.VOLTAGE_COMPENSATION_SATURATION
        );
        motor.setVoltage(voltage);
    }
}
