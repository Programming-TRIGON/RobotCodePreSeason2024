package frc.trigon.robot.subsystems.turret.simulationturret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.turret.TurretIO;
import frc.trigon.robot.subsystems.turret.TurretInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;

public class SimulationTurretIO extends TurretIO {
    private final DCMotorSim motorSim = SimulationTurretConstants.MOTOR;
    private double motorVoltage;

    @Override
    protected void updateInputs(TurretInputsAutoLogged inputs) {
        motorSim.update(RobotConstants.PERIODIC_TIME_SECONDS);

        inputs.motorVoltage = motorVoltage;
        inputs.motorAngleDegrees = getMotorAngleDegrees();
        inputs.motorVelocityDegreesPerSecond = getMotorVelocityDegreesPerSecond();
    }

    @Override
    protected void setTargetAngle(Rotation2d targetAngle) {
        double voltage = calculateTargetMotorVoltage(targetAngle);
        motorVoltage = voltage;
        setMotorVoltageFromPower(voltage);
    }

    @Override
    protected void stop() {
        motorVoltage = 0;
        setMotorVoltageFromPower(0);
    }

    private double calculateTargetMotorVoltage(Rotation2d targetAngle) {
        double pidOutput = SimulationTurretConstants.MOTOR_SIMULATION.profiledPIDController.calculate(targetAngle.getRotations());
        SimulationTurretConstants.MOTOR_SIMULATION.profiledPIDController.setGoal(0);
        double feedforward = SimulationTurretConstants.MOTOR_SIMULATION.profiledPIDController.calculate(SimulationTurretConstants.MOTOR_SIMULATION.profiledPIDController.getGoal().velocity);
        return pidOutput + feedforward;
    }

    public void setMotorVoltageFromPower(double power) {
        double motorVoltage = MathUtil.clamp(
                Conversions.compensatedPowerToVoltage(power, SimulationTurretConstants.VOLTAGE_COMPENSATION_SATURATION),
                -SimulationTurretConstants.VOLTAGE_COMPENSATION_SATURATION,
                SimulationTurretConstants.VOLTAGE_COMPENSATION_SATURATION
        );
        motorSim.setInputVoltage(motorVoltage);
    }

    private double getMotorAngleDegrees() {
        return Units.radiansToDegrees(motorSim.getAngularPositionRad());
    }

    private double getMotorVelocityDegreesPerSecond() {
        return Units.radiansToDegrees(motorSim.getAngularVelocityRadPerSec());
    }
}
