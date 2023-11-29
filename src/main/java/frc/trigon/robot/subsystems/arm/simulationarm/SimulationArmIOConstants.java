package frc.trigon.robot.subsystems.arm.simulationarm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class SimulationArmIOConstants {
    private static final int
            ANGLE_MOTOR_AMOUNT = 2,
            ELEVATOR_MOTOR_AMOUNT = 2;
    private static final DCMotor
            MASTER_ANGLE_MOTOR_GEARBOX = DCMotor.getNEO(ANGLE_MOTOR_AMOUNT),
            MASTER_ELEVATOR_MOTOR_GEARBOX = DCMotor.getNEO(ELEVATOR_MOTOR_AMOUNT);
    private static final double
            ANGLE_GEAR_RATIO = 12.8;
    private static final double
            MIN_ELEVATOR_HEIGHT_METERS = 0,
            MAX_ELEVATOR_HEIGHT_METERS = 1;
    private static final Rotation2d
            STARTING_ANGLE_RADIANS = Rotation2d.fromDegrees(5),
            MIN_ANGLE_RADIANS = Rotation2d.fromDegrees(5),
            MAX_ANGLE_RADIANS = Rotation2d.fromDegrees(5);
    private static final boolean
            ANGLE_SIMULATE_GRAVITY = true,
            ELEVATOR_SIMULATE_GRAVITY = true;
    private static final double
            RETRACTED_ARM_LENGTH_METERS = 1,
            ELEVATOR_STARTING_HEIGHT_METERS = 1;
    private static final double
            ARM_MASS = 1,
            DRUM_RADIUS_METERS = 1;
    static final double METERS_PER_REVOLUTION = 2048;

    static final SingleJointedArmSim ANGLE_MOTOR = new SingleJointedArmSim(
            MASTER_ANGLE_MOTOR_GEARBOX,
            ANGLE_GEAR_RATIO,
            SingleJointedArmSim.estimateMOI(RETRACTED_ARM_LENGTH_METERS, ARM_MASS),
            RETRACTED_ARM_LENGTH_METERS,
            MIN_ANGLE_RADIANS.getRadians(),
            MAX_ANGLE_RADIANS.getRadians(),
            ANGLE_SIMULATE_GRAVITY,
            STARTING_ANGLE_RADIANS.getRadians()
    );
    static final ElevatorSim ELEVATOR_MOTOR = new ElevatorSim(
            MASTER_ELEVATOR_MOTOR_GEARBOX,
            SingleJointedArmSim.estimateMOI(RETRACTED_ARM_LENGTH_METERS, ARM_MASS),
            ARM_MASS,
            DRUM_RADIUS_METERS,
            MIN_ELEVATOR_HEIGHT_METERS,
            MAX_ELEVATOR_HEIGHT_METERS,
            ELEVATOR_SIMULATE_GRAVITY,
            ELEVATOR_STARTING_HEIGHT_METERS
    );

    private static final double
            ANGLE_P = 1,
            ANGLE_I = 0,
            ANGLE_D = 0,
            ELEVATOR_P = 1,
            ELEVATOR_I = 0,
            ELEVATOR_D = 0;
    static final PIDController
            ANGLE_PID_CONTROLLER = new PIDController(ANGLE_P, ANGLE_I, ANGLE_D),
            ELEVATOR_PID_CONTROLLER = new PIDController(ELEVATOR_P, ELEVATOR_I, ELEVATOR_D);

    private static final double
            ANGLE_MOTOR_KS = 0,
            ANGLE_MOTOR_KV = 0,
            ANGLE_MOTOR_KA = 0,
            ANGLE_MOTOR_KG = 0,
            ELEVATOR_MOTOR_KS = 0,
            ELEVATOR_MOTOR_KV = 0,
            ELEVATOR_MOTOR_KA = 0,
            ELEVATOR_MOTOR_KG = 0;
    static final ArmFeedforward ANGLE_FEEDFORWARD = new ArmFeedforward(
            ANGLE_MOTOR_KS, ANGLE_MOTOR_KG, ANGLE_MOTOR_KV, ANGLE_MOTOR_KA
    );
    static final ElevatorFeedforward ELEVATOR_FEEDFORWARD = new ElevatorFeedforward(
            ELEVATOR_MOTOR_KS, ELEVATOR_MOTOR_KG, ELEVATOR_MOTOR_KV, ELEVATOR_MOTOR_KA
    );
}