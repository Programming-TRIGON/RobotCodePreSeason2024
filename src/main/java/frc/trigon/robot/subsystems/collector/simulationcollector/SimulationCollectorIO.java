package frc.trigon.robot.subsystems.collector.simulationcollector;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.collector.CollectorIO;
import frc.trigon.robot.subsystems.collector.CollectorInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;

public class SimulationCollectorIO extends CollectorIO {
    private final DCMotorSim motor = SimulationCollectorIOConstants.MOTOR;
    private double motorVoltage;

    @Override
    protected void updateInputs(CollectorInputsAutoLogged inputs) {
        motor.update(RobotConstants.PERIODIC_TIME_SECONDS);

        inputs.motorVoltage = motorVoltage;
        inputs.motorCurrent = motor.getCurrentDrawAmps();
    }

    protected void setVoltage(double power) {
        this.motorVoltage = Conversions.compensatedPowerToVoltage(power, SimulationCollectorIOConstants.VOLTAGE_COMPENSATION_SATURATION);
        motor.setInputVoltage(motorVoltage);
    }

    @Override
    protected void stop() {
        motor.setInputVoltage(0);
    }
}
