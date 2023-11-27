package frc.trigon.robot.subsystems.arm.simulationarm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class SimulationArmIOConstants {
    private static final int MOTOR_AMOUNT = 1;
    private static final DCMotor
            ANGLE_MOTOR_GEARBOX = DCMotor.getBag(MOTOR_AMOUNT),
            ELEVATOR_MOTOR_GEARBOX = DCMotor.getBag(MOTOR_AMOUNT);
    private static final double MOMENT_OF_INERTIA = 1;
    private static final double GEAR_RATIO = 12.8;
    private static final double
            KA = 1,
            KV = 1;
    private static final double
            MIN_HEIGHT_METERS = 0,
            MAX_HEIGHT_METERS = 1;
    private static final double
            STARTING_ANGLE_RADS = 0,
            MIN_ANGLE_RADS = 0,
            MAX_ANGLE_RADS = 1;
    private static final boolean SIMULATE_GRAVITY = true;
    private static final double
            ANGLE_ARM_LENGTH_METERS = 1,
            ELEVATOR_STARTING_HEIGHT_METERS = 1;
    static final SingleJointedArmSim ANGLE_MOTOR = new SingleJointedArmSim(
            ANGLE_MOTOR_GEARBOX,
            GEAR_RATIO,
            MOMENT_OF_INERTIA,
            ANGLE_ARM_LENGTH_METERS,
            MIN_ANGLE_RADS,
            MAX_ANGLE_RADS,
            SIMULATE_GRAVITY,
            STARTING_ANGLE_RADS
    );
    static final ElevatorSim ELEVATOR_MOTOR = new ElevatorSim(
            KA,
            KV,
            ELEVATOR_MOTOR_GEARBOX,
            MIN_HEIGHT_METERS,
            MAX_HEIGHT_METERS,
            SIMULATE_GRAVITY,
            ELEVATOR_STARTING_HEIGHT_METERS
    );
}
