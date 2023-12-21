package frc.trigon.robot.subsystems.turret.kablamaturret;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.subsystems.turret.TurretIO;
import frc.trigon.robot.subsystems.turret.TurretInputsAutoLogged;

public class KablamaTurretIO extends TurretIO {
    private final TalonFX motor = KablamaTurretConstants.MOTOR;
    private final MotionMagicVoltage PositionVoltageRequest = new MotionMagicVoltage(0).withSlot(0);

    @Override
    protected void updateInputs(TurretInputsAutoLogged inputs) {
        inputs.motorPositionDegrees = getPosition().getDegrees();
        inputs.motorVelocityDegreesPerSecond = motor.getVelocity().getValue();
        inputs.motorVoltage = getVelocityDegreesPerSecond();
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
        return Rotation2d.fromRotations(motor.getPosition().refresh().getValue());
    }

    private double getVelocityDegreesPerSecond() {
        return Units.rotationsToDegrees(motor.getVelocity().refresh().getValue());
    }
}
