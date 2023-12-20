package frc.trigon.robot.subsystems.turret.simulationturret;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimulationTurretConstants {
    static final double VOLTAGE_COMPENSATION_SATURATION = 12;

    private static final double
            MAX_MOTOR_VELOCITY = 200,
            MAX_MOTOR_ACCELERATION = 3;
    private static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(MAX_MOTOR_VELOCITY, MAX_MOTOR_ACCELERATION);
    private static final int MOTOR_AMOUNT = 1;
    private static final DCMotor MOTOR_GEARBOX = DCMotor.getFalcon500Foc(MOTOR_AMOUNT);
    private static final double GEAR_RATIO = 100;
    private static final double MOMENT_OF_INERTIA = 0.003;
    static final DCMotorSim MOTOR = new DCMotorSim(
            MOTOR_GEARBOX,
            GEAR_RATIO,
            MOMENT_OF_INERTIA
    );

    private static final double
            P = 0,
            I = 0,
            D = 0;
    static final ProfiledPIDController PROFILED_PID_CONTROLLER = new ProfiledPIDController(P, I, D, CONSTRAINTS);

    private static final double
            KS = 0,
            KV = 0,
            KA = 0;
    static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(KS, KV, KA);
}
