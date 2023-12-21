package frc.trigon.robot.subsystems.turret.simulationturret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.turret.TurretIO;
import frc.trigon.robot.subsystems.turret.TurretInputsAutoLogged;

public class SimulationTurretIO extends TurretIO {
    private final DCMotorSim motor = SimulationTurretConstants.MOTOR;
    private double voltage = 0;
    
    @Override
    protected void updateInputs(TurretInputsAutoLogged inputs) {
        motor.update(RobotConstants.PERIODIC_TIME_SECONDS);
        inputs.motorPositionDegrees = Units.radiansToDegrees(motor.getAngularPositionRad());
        inputs.motorVelocityDegreesPerSecond = Units.radiansToDegrees(motor.getAngularVelocityRadPerSec());
        inputs.motorVoltage = voltage;
    }

    @Override
    protected void setTargetAngle(Rotation2d targetAngle) {
        this.voltage = calculateTargetAngle(targetAngle);
        motor.setInputVoltage(calculateTargetAngle(targetAngle));
    }

    @Override
    protected void stop() {
        this.voltage = 0;
        motor.setInputVoltage(0);
    }

    private double calculateTargetAngle(Rotation2d targetAngle) {
        double pidOutput = SimulationTurretConstants.PROFILED_PID_CONTROLLER.calculate((targetAngle.getDegrees()));
        double feedforwardOutput = SimulationTurretConstants.FEEDFORWARD.calculate(motor.getAngularVelocityRadPerSec());
        return pidOutput + feedforwardOutput;
    }
}
