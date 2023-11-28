package frc.trigon.robot.subsystems.collector.simulationcollector;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimulationCollectorIOConstants {
    private static final int MOTOR_AMOUNT = 1;
    private static final DCMotor MOTOR_GEARBOX = DCMotor.getBag(MOTOR_AMOUNT);
    private static final double MOMENT_OF_INERTIA = 0.003;
    private static final double GEAR_RATIO = 12.8;
    static final int VOLTAGE_COMPENSATION_SATURATION = 12;
    static final DCMotorSim MOTOR = new DCMotorSim(
            MOTOR_GEARBOX,
            GEAR_RATIO,
            MOMENT_OF_INERTIA
    );
}
