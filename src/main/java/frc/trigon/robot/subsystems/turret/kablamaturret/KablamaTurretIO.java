package frc.trigon.robot.subsystems.turret.kablamaturret;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.subsystems.turret.TurretIO;
import frc.trigon.robot.subsystems.turret.TurretInputsAutoLogged;

public class KablamaTurretIO extends TurretIO {
    private final TalonFX motor = KablamaTurretConstants.MOTOR;
    private final MotionMagicVoltage PositionVoltageRequest = new MotionMagicVoltage(0).withEnableFOC(KablamaTurretConstants.FOC_ENABLED);

    @Override
    protected void updateInputs(TurretInputsAutoLogged inputs) {
        inputs.motorPositionDegrees = getPosition().getDegrees();
        inputs.motorVelocityDegreesPerSecond = getVelocityDegreesPerSecond();
        inputs.motorVoltage = motor.getMotorVoltage().refresh().getValue();
    }

    @Override
    protected void setTargetAngle(Rotation2d targetAngle) {
        motor.setControl(PositionVoltageRequest.withPosition(targetAngle.getRotations()));
    }

    @Override
    protected void stop() {
        motor.stopMotor();
    }

    private Rotation2d getPosition() {
        return Rotation2d.fromRotations(KablamaTurretConstants.TURRET_POSITION_SIGNAL.refresh().getValue());
    }

    private double getVelocityDegreesPerSecond() {
        return Units.rotationsToDegrees(KablamaTurretConstants.TURRET_VELOCITY_SIGNAL.refresh().getValue());
    }
}
