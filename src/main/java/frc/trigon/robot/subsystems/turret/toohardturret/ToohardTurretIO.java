package frc.trigon.robot.subsystems.turret.toohardturret;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.subsystems.turret.TurretIO;
import frc.trigon.robot.subsystems.turret.TurretInputsAutoLogged;

public class ToohardTurretIO extends TurretIO {
    private final TalonFX motor = ToohardTurretConstants.MOTOR;
    private final PositionVoltage positionRequest = new PositionVoltage(0, 0, ToohardTurretConstants.FOC_ENABLED, 0, 0, false);

    @Override
    protected void updateInputs(TurretInputsAutoLogged inputs) {
        inputs.motorVoltage = motor.getMotorVoltage().refresh().getValue();
        inputs.motorAngleDegrees = ToohardTurretConstants.ENCODER_POSITION_SIGNAL.refresh().getValue();
        inputs.motorVelocityDegreesPerSecond = ToohardTurretConstants.ENCODER_VELOCITY_SIGNAL.refresh().getValue();
    }

    @Override
    protected void setTargetAnglePosition(Rotation2d targetAngle) {
        motor.setControl(positionRequest.withPosition(targetAngle.getDegrees()));
    }
}
