package frc.trigon.robot.subsystems.turret.simulationturret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.trigon.robot.subsystems.turret.TurretIO;

public class SimulationTurretConstants extends TurretIO {
    private static final int MOTOR_AMOUNT = 1;
    private static final DCMotor MOTOR_GEARBOX = DCMotor.getFalcon500Foc(MOTOR_AMOUNT);
    private static final double MOMENT_OF_INERTIA = 0.003;
    private static final double GEAR_RATIO = 100;

    static final DCMotorSim MOTOR = new DCMotorSim(
            MOTOR_GEARBOX,
            GEAR_RATIO,
            MOMENT_OF_INERTIA
    );

    private static final double
            P = 1,
            I = 0,
            D = 0,
            KS = 0,
            KV = 0,
            KA = 0;
    static final PIDController PID_CONTROLLER = new PIDController(P, I, D);
    static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(KS, KV, KA);
}
