package frc.trigon.robot.subsystems.turret.simulationturret;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class SimulationTurretConstants {
    static final double VOLTAGE_COMPENSATION_SATURATION = 12;
    private static final int MOTOR_AMOUNT = 1;
    private static final DCMotor MOTOR_GEARBOX = DCMotor.getFalcon500(MOTOR_AMOUNT);
    private static final double GEAR_RATIO = 100;
    private static final double MOMENT_OF_INERTIA = 0.003;
    private static final double ARM_LENGTH = 0;
    private static final Rotation2d
            MINIMUM_ANGLE = Rotation2d.fromDegrees(-200),
            MAXIMUM_ANGLE = Rotation2d.fromDegrees(200),
            START_ANGLE = Rotation2d.fromDegrees(0);
    private static final boolean SIMULATE_GRAVITY = true;
    static final SingleJointedArmSim MOTOR = new SingleJointedArmSim(
            MOTOR_GEARBOX,
            GEAR_RATIO,
            MOMENT_OF_INERTIA,
            ARM_LENGTH,
            MINIMUM_ANGLE.getRadians(),
            MAXIMUM_ANGLE.getRadians(),
            SIMULATE_GRAVITY,
            START_ANGLE.getRadians()
    );

    private static final double
            P = 0,
            I = 0,
            D = 0;
    static final PIDController PID_CONTROLLER = new PIDController(P, I, D);

    private static final double
            KS = 0,
            KG = 0,
            KV = 0,
            KA = 0;
    static final ArmFeedforward FEEDFORWARD = new ArmFeedforward(KS, KG, KV, KA);
}
