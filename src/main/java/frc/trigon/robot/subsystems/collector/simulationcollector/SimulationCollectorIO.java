package frc.trigon.robot.subsystems.collector.simulationcollector;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.collector.CollectorIO;
import frc.trigon.robot.subsystems.collector.CollectorInputsAutoLogged;

public class SimulationCollectorIO extends CollectorIO {
    private final DCMotorSim motor = SimulationCollectorIOConstants.MOTOR;
    private double voltage;

    @Override
    protected void updateInputs(CollectorInputsAutoLogged inputs) {
        motor.update(RobotConstants.PERIODIC_TIME_SECONDS);
        inputs.motorVoltage = voltage;
        inputs.motorCurrent = motor.getCurrentDrawAmps();
    }

    protected void setVoltage(double voltage) {
        this.voltage = voltage;
        motor.setInputVoltage(voltage);
    }

    @Override
    protected void stop() {
        motor.setInputVoltage(0);
    }
}
