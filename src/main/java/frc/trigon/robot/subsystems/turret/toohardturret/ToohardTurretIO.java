package frc.trigon.robot.subsystems.turret.toohardturret;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.subsystems.turret.TurretIO;
import frc.trigon.robot.subsystems.turret.TurretInputsAutoLogged;

public class ToohardTurretIO extends TurretIO {
    private final TalonFX motor = ToohardTurretConstants.MOTOR;

    @Override
    protected void updateInputs(TurretInputsAutoLogged inputs) {
        inputs.motorVoltage = motor.getMotorVoltage().refresh().getValue();
        inputs.motorAngleDegrees = ToohardTurretConstants.ENCODER_POSITION_SIGNAL.refresh().getValue();
        inputs.motorVelocityDegreesPerSecond = ToohardTurretConstants.ENCODER_VELOCITY_SIGNAL.refresh().getValue();
    }

    @Override
    protected void setTargetAngleState(TrapezoidProfile.State targetState) {
        motor.setVoltage(calculateVoltageFromState(targetState));
    }

    @Override
    protected void stopMotor() {

    }

    private double calculateVoltageFromState(TrapezoidProfile.State targetState) {
        double pidOutput = ToohardTurretConstants.PID_CONTROLLER.calculate(
                ToohardTurretConstants.ENCODER_POSITION_SIGNAL.refresh().getValue(),
                targetState.position
        );
        double feedforward = ToohardTurretConstants.FEEDFORWARD.calculate(
                Units.degreesToRadians(targetState.position),
                targetState.velocity
        );
        return pidOutput + feedforward;
    }
}
