package frc.trigon.robot.subsystems.turret.kablamaturret;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.subsystems.turret.TurretIO;
import frc.trigon.robot.subsystems.turret.TurretInputsAutoLogged;

public class KablamaTurretIO extends TurretIO {
    private final TalonFX motor = KablamaTurretConstants.MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0, KablamaTurretConstants.FOC_ENABLED, false);

    @Override
    protected void updateInputs(TurretInputsAutoLogged inputs) {
        inputs.motorPositionDegrees = getPositionDegrees();
    }

    @Override
    protected void calculateMotorVoltage(double voltage) {
        setMotorVoltage(voltage);
    }

    @Override
    protected void stop() {
        motor.stopMotor();
    }

    private double getPositionDegrees() {
        return Units.rotationsToDegrees(KablamaTurretConstants.TURRET_POSITION_SIGNAL.refresh().getValue());
    }

    private void setMotorVoltage(double voltage) {
        motor.setControl(voltageRequest.withOutput(voltage));
    }
}
