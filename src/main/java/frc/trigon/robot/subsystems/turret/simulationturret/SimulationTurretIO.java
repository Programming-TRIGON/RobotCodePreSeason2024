package frc.trigon.robot.subsystems.turret.simulationturret;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.trigon.robot.subsystems.turret.TurretIO;
import frc.trigon.robot.subsystems.turret.TurretInputsAutoLogged;

public class SimulationTurretIO extends TurretIO {
    private final DCMotorSim motor = SimulationTurretConstants.MOTOR;
    
    @Override
    protected void updateInputs(TurretInputsAutoLogged inputs) {
        inputs.motorPositionDegrees = Units.radiansToDegrees(motor.getAngularPositionRad());
    }

    @Override
    protected void calculateMotorVoltage(double voltage) {
        motor.setInputVoltage(SimulationTurretConstants.PID_CONTROLLER.calculate(voltage));
    }

    @Override
    protected void stop() {
        motor.setInputVoltage(0);
    }
}
