package frc.trigon.robot.subsystems.turret.kablamaturret;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.subsystems.turret.TurretIO;
import frc.trigon.robot.subsystems.turret.TurretInputsAutoLogged;

public class KablamaTurretIO extends TurretIO {
    private final TalonFX motor = KablamaTurretConstants.MOTOR;
    private final PositionVoltage voltageRequest = new PositionVoltage(0);

    @Override
    protected void updateInputs(TurretInputsAutoLogged inputs) {
        inputs.motorPositionDegrees = getPositionDegrees();
    }

    @Override
    protected void calculateMotorVoltage(double voltage) {
        motor.setControl(voltageRequest.withPosition(voltage));
    }

    @Override
    protected void stop() {
        motor.stopMotor();
    }

    private double getPositionDegrees() {
        return Units.rotationsToDegrees(KablamaTurretConstants.TURRET_POSITION_SIGNAL.refresh().getValue());
    }
}
