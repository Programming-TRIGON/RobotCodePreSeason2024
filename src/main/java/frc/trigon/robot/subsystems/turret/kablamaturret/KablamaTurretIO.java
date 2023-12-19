package frc.trigon.robot.subsystems.turret.kablamaturret;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.subsystems.turret.TurretIO;
import frc.trigon.robot.subsystems.turret.TurretInputsAutoLogged;

public class KablamaTurretIO extends TurretIO {
    private final TalonFX motor = KablamaTurretConstants.MOTOR;
    private final PositionVoltage PositionVoltageRequest = new PositionVoltage(0);

    @Override
    protected void updateInputs(TurretInputsAutoLogged inputs) {
        inputs.motorPositionDegrees = getPosition().getDegrees();
    }

    @Override
    protected void setTargetAngle(Rotation2d targetAngle) {
        motor.setControl(PositionVoltageRequest.withPosition(targetAngle.getDegrees()));
    }

    @Override
    protected void stop() {
        motor.stopMotor();
    }

    private Rotation2d getPosition() {
        return Rotation2d.fromRotations(KablamaTurretConstants.TURRET_POSITION_SIGNAL.refresh().getValue());
    }
}
