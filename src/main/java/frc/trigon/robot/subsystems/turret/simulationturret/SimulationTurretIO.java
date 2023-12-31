package frc.trigon.robot.subsystems.turret.simulationturret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.simulationwrapper.DCMotorSimulation;
import frc.trigon.robot.subsystems.turret.TurretIO;
import frc.trigon.robot.subsystems.turret.TurretInputsAutoLogged;

public class SimulationTurretIO extends TurretIO {
    private final DCMotorSimulation DCMotorSimulation = SimulationTurretConstants.MOTOR_SIMULATION;

    private double motorVoltage;

    @Override
    protected void updateInputs(TurretInputsAutoLogged inputs) {
        DCMotorSimulation.motorSim.update(RobotConstants.PERIODIC_TIME_SECONDS);

        inputs.motorVoltage = motorVoltage;
        inputs.motorAngleDegrees = getMotorAngleDegrees();
        inputs.motorVelocityDegreesPerSecond = getMotorVelocityDegreesPerSecond();
    }

    @Override
    protected void setTargetAngle(Rotation2d targetAngle) {
        double voltage = SimulationTurretConstants.MOTOR_SIMULATION.calculateTargetMotorVoltage(targetAngle);
        motorVoltage = voltage;
        DCMotorSimulation.setMotorVoltageFromPower(voltage);
    }

    @Override
    protected void stop() {
        motorVoltage = 0;
        DCMotorSimulation.setMotorVoltageFromPower(0);
    }

    private double getMotorAngleDegrees() {
        return Units.radiansToDegrees(DCMotorSimulation.motorSim.getAngularPositionRad());
    }

    private double getMotorVelocityDegreesPerSecond() {
        return Units.radiansToDegrees(DCMotorSimulation.motorSim.getAngularVelocityRadPerSec());
    }
}
