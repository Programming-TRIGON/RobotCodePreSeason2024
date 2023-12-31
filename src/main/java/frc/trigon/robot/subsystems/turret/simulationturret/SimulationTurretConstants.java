package frc.trigon.robot.subsystems.turret.simulationturret;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.trigon.robot.simulationwrapper.DCMotorSimulation;

public class SimulationTurretConstants {
    public static final double VOLTAGE_COMPENSATION_SATURATION = 12;

    private static final int MOTOR_AMOUNT = 1;
    private static final DCMotor MOTOR_GEARBOX = DCMotor.getFalcon500Foc(MOTOR_AMOUNT);
    private static final double GEAR_RATIO = 100;
    private static final double MOMENT_OF_INERTIA = 0.003;
    public static final DCMotorSim MOTOR = new DCMotorSim(
            MOTOR_GEARBOX,
            GEAR_RATIO,
            MOMENT_OF_INERTIA
    );
    private static final double
            MAX_MOTOR_VELOCITY = 5,
            MAX_MOTOR_ACCELERATION = 3;
    private static final double
            P = 0,
            I = 0,
            D = 0;
    private static final double
            KS = 0,
            KV = 0,
            KA = 0;

    public static DCMotorSimulation MOTOR_SIMULATION = new DCMotorSimulation();

    static {
        MOTOR_SIMULATION.motorSim = MOTOR;
        MOTOR_SIMULATION.constraints = new TrapezoidProfile.Constraints(MAX_MOTOR_VELOCITY, MAX_MOTOR_ACCELERATION);
        MOTOR_SIMULATION.setPID(P, I, D);
        MOTOR_SIMULATION.setFeedforward(KS, KV, KA);
    }
}
