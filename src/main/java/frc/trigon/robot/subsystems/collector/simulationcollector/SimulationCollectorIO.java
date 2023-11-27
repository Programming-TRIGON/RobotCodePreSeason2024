package frc.trigon.robot.subsystems.collector.simulationcollector;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.trigon.robot.subsystems.collector.CollectorIO;
import frc.trigon.robot.subsystems.collector.CollectorInputsAutoLogged;

public class SimulationCollectorIO extends CollectorIO {
    private final DCMotorSim dcMotorSim = SimulationCollectorIOConstants.DC_MOTOR_SIM;

    @Override
    protected void updateInputs(CollectorInputsAutoLogged inputs) {
        inputs.motorVoltage = dcMotorSim.getOutput().get(0, 0);
        inputs.motorCurrent = dcMotorSim.getCurrentDrawAmps();
    }

    @Override
    protected void setPower(double power) {
        dcMotorSim.setInputVoltage(power);
    }

    @Override
    protected void stop() {
        dcMotorSim.setInputVoltage(0);
    }
}
