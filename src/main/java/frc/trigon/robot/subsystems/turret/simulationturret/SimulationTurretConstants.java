package frc.trigon.robot.subsystems.turret.simulationturret;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.simulationwrapper.MotorSimulation;
import frc.trigon.robot.simulationwrapper.MotorSimulationConfiguration;

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
    public static final MotorSimulation MOTOR_SIMULATION = new MotorSimulation();
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

    static {
        MotorSimulationConfiguration config = new MotorSimulationConfiguration();
        config.maxVelocity = MAX_MOTOR_VELOCITY;
        config.maxAcceleration = MAX_MOTOR_ACCELERATION;
        config.p = P;
        config.i = I;
        config.d = D;
        config.ks = KS;
        config.kv = KV;
        config.ka = KA;
        MOTOR_SIMULATION.applyConfigurations(config);
//        SysIdRoutine sysIdRoutine = new SysIdRoutine(
//                new SysIdRoutine.Config()
//        );
    }
}
