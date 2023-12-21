package frc.trigon.robot.subsystems.turret.toohardturret;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.subsystems.turret.TurretIO;
import frc.trigon.robot.subsystems.turret.TurretInputsAutoLogged;

public class ToohardTurretIO extends TurretIO {
    private final TalonFX motor = ToohardTurretConstants.MOTOR;
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(ToohardTurretConstants.IS_FOC_ENABLED);

    @Override
    protected void updateInputs(TurretInputsAutoLogged inputs) {
        inputs.motorVoltage = motor.getMotorVoltage().refresh().getValue();
        inputs.motorAngleDegrees = ToohardTurretConstants.ENCODER_POSITION_SIGNAL_ANGLE.refresh().getValue();
        inputs.motorVelocityDegreesPerSecond = ToohardTurretConstants.ENCODER_VELOCITY_SIGNAL_ANGLE.refresh().getValue();
    }

    @Override
    protected void setTargetAngle(Rotation2d targetAngle) {
        motor.setControl(positionRequest.withPosition(targetAngle.getDegrees()));
    }

    @Override
    protected void stop() {
        motor.stopMotor();
    }
}
