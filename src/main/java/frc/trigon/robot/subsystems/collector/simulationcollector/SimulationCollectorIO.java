package frc.trigon.robot.subsystems.collector.simulationcollector;

import edu.wpi.first.math.MathUtil;
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

    @Override
    protected void setPower(double power) {
        double voltage = Conversions.compensatedPowerToVoltage(power, SimulationCollectorIOConstants.VOLTAGE_COMPENSATION_SATURATION);
        this.motorVoltage = MathUtil.clamp(voltage, -SimulationCollectorIOConstants.VOLTAGE_COMPENSATION_SATURATION, SimulationCollectorIOConstants.VOLTAGE_COMPENSATION_SATURATION);
        motor.setInputVoltage(this.motorVoltage);
    }

    @Override
    protected void stop() {
        setPower(0);
    }
}
