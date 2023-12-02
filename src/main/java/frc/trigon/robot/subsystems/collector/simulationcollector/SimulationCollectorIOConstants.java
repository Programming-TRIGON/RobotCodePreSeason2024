package frc.trigon.robot.subsystems.collector.simulationcollector;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;

public class SimulationCollectorIOConstants {
    private static final int MOTOR_AMOUNT = 1;
    private static final DCMotor MOTOR_GEARBOX = DCMotor.getBag(MOTOR_AMOUNT);
    private static final double MOMENT_OF_INERTIA = 0.003;
    private static final double GEAR_RATIO = 12.8;
    static final int VOLTAGE_COMPENSATION_SATURATION = 12;
    private static final double
            MECHANISM2D_WIDTH = 1,
            MECHANISM2D_HEIGHT = 1;

    static final Mechanism2d COLLECTOR_MECHANISM = new Mechanism2d(MECHANISM2D_WIDTH, MECHANISM2D_HEIGHT);

    static final DCMotorSim MOTOR = new DCMotorSim(
            MOTOR_GEARBOX,
            GEAR_RATIO,
            MOMENT_OF_INERTIA
    );
}
