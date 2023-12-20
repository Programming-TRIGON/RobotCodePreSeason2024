package frc.trigon.robot.subsystems.turret.simulationturret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.turret.TurretIO;
import frc.trigon.robot.subsystems.turret.TurretInputsAutoLogged;

public class SimulationTurretIO extends TurretIO {
    private final DCMotorSim motor = SimulationTurretConstants.MOTOR;

    private double motorVoltage;

    @Override
    protected void updateInputs(TurretInputsAutoLogged inputs) {
        motor.update(RobotConstants.PERIODIC_TIME_SECONDS);

        inputs.motorVoltage = motorVoltage;
        inputs.motorAngleDegrees = Units.radiansToDegrees(motor.getAngularPositionRad());
        inputs.motorVelocityDegreesPerSecond = Units.radiansToDegrees(motor.getAngularVelocityRadPerSec());
    }

    @Override
    protected void setTargetAngle(Rotation2d targetAngle) {
        setMotorVoltageFromPower(SimulationTurretConstants.PID_CONTROLLER.calculate(targetAngle.getDegrees()));
    }

    private void setMotorVoltageFromPower(double power) {
        motorVoltage = MathUtil.clamp(
                power * SimulationTurretConstants.VOLTAGE_COMPENSATION_SATURATION,
                -SimulationTurretConstants.VOLTAGE_COMPENSATION_SATURATION,
                SimulationTurretConstants.VOLTAGE_COMPENSATION_SATURATION
        );
        motor.setInputVoltage(motorVoltage);
    }
}
