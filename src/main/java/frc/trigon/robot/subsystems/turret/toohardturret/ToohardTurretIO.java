package frc.trigon.robot.subsystems.turret.toohardturret;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.subsystems.turret.TurretIO;
import frc.trigon.robot.subsystems.turret.TurretInputsAutoLogged;

public class ToohardTurretIO extends TurretIO {
    private final TalonFX motor = ToohardTurretConstants.MOTOR;
    private final StatusSignal<Double> MOTOR_VOLTAGE_STATUS_SIGNAL = motor.getMotorVoltage();
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(ToohardTurretConstants.FOC_ENABLED);

    @Override
    protected void updateInputs(TurretInputsAutoLogged inputs) {
        inputs.motorVoltage = MOTOR_VOLTAGE_STATUS_SIGNAL.refresh().getValue();
        inputs.motorAngleDegrees = ToohardTurretConstants.ENCODER_POSITION_SIGNAL.refresh().getValue();
        inputs.motorVelocityDegreesPerSecond = ToohardTurretConstants.ENCODER_VELOCITY_SIGNAL.refresh().getValue();
    }

    @Override
    protected void setTargetAngle(Rotation2d targetAngle) {
        motor.setControl(positionRequest.withPosition(targetAngle.getRotations()));
    }

    @Override
    protected void stop() {
        motor.stopMotor();
    }
}
