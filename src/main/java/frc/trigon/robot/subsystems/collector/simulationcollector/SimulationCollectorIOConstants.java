package frc.trigon.robot.subsystems.collector.simulationcollector;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimulationCollectorIOConstants {
    private static final double
            NOMINAL_VOLTAGE_VOLTS = 1,
            STALL_TORQUE_NEWTON_METERS = 1,
            STALL_CURRENT_AMPS = 1,
            FREE_CURRENT_AMPS = 1,
            MOMENT_OF_INERTIA = 1;
    private static final int NUM_MOTORS = 1;
    private static final DCMotor GEARBOX = DCMotor.getFalcon500(NUM_MOTORS);
    private static final double
            JKG_METERS_SQUARED = 1,
            GEARING = 1;
    static final DCMotorSim MOTOR =
            new DCMotorSim(
                    GEARBOX,
                    GEARING,
                    JKG_METERS_SQUARED
            );
}
