package frc.trigon.robot.subsystems.arm.simulationarm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class SimulationArmIOConstants {
    private static final int MOTOR_AMOUNT = 1;
    private static final DCMotor
            MASTER_ANGLE_MOTOR_GEARBOX = DCMotor.getNEO(MOTOR_AMOUNT),
            FOLLOWER_ANGLE_MOTOR_GEARBOX = DCMotor.getNEO(MOTOR_AMOUNT),
            MASTER_ELEVATOR_MOTOR_GEARBOX = DCMotor.getNEO(MOTOR_AMOUNT),
            FOLLOWER_ELEVATOR_MOTOR_GEARBOX = DCMotor.getNEO(MOTOR_AMOUNT);
    private static final double MOMENT_OF_INERTIA = 0.003;
    private static final double
            ANGLE_GEAR_RATIO = 12.8,
            ELEVATOR_GEAR_RATIO = 12.8;
    private static final double
            KA = 1,
            KV = 1;
    private static final double
            MIN_ELEVATOR_HEIGHT_METERS = 0,
            MAX_ELEVATOR_HEIGHT_METERS = 1;
    private static final Rotation2d
            STARTING_ANGLE_RADIANS = Rotation2d.fromRadians(5),
            MIN_ANGLE_RADIANS = Rotation2d.fromRadians(5),
            MAX_ANGLE_RADIANS = Rotation2d.fromRadians(5);
    private static final boolean
            ANGLE_SIMULATE_GRAVITY = true,
            ELEVATOR_SIMULATE_GRAVITY = true;
    private static final double
            RETRACTED_ARM_LENGTH_METERS = 1,
            ELEVATOR_STARTING_HEIGHT_METERS = 1;
    private static final double
            CARRIAGE_MASS_KG = 1,
            DRUM_RADIUS_METERS = 1;

    static final SingleJointedArmSim MASTER_ANGLE_MOTOR = new SingleJointedArmSim(
            MASTER_ANGLE_MOTOR_GEARBOX,
            ANGLE_GEAR_RATIO,
            MOMENT_OF_INERTIA,
            RETRACTED_ARM_LENGTH_METERS,
            MIN_ANGLE_RADIANS.getRadians(),
            MAX_ANGLE_RADIANS.getRadians(),
            ANGLE_SIMULATE_GRAVITY,
            STARTING_ANGLE_RADIANS.getRadians()
    );
    static final SingleJointedArmSim FOLLOWER_ANGLE_MOTOR = new SingleJointedArmSim(
            FOLLOWER_ANGLE_MOTOR_GEARBOX,
            ANGLE_GEAR_RATIO,
            SingleJointedArmSim.estimateMOI(RETRACTED_ARM_LENGTH_METERS, CARRIAGE_MASS_KG),
            RETRACTED_ARM_LENGTH_METERS,
            MIN_ANGLE_RADIANS.getRadians(),
            MAX_ANGLE_RADIANS.getRadians(),
            ANGLE_SIMULATE_GRAVITY,
            STARTING_ANGLE_RADIANS.getRadians()
    );
    static final ElevatorSim MASTER_ELEVATOR_MOTOR = new ElevatorSim(
            MASTER_ELEVATOR_MOTOR_GEARBOX,
            SingleJointedArmSim.estimateMOI(RETRACTED_ARM_LENGTH_METERS, CARRIAGE_MASS_KG),
            CARRIAGE_MASS_KG,
            DRUM_RADIUS_METERS,
            MIN_ELEVATOR_HEIGHT_METERS,
            MAX_ELEVATOR_HEIGHT_METERS,
            ELEVATOR_SIMULATE_GRAVITY,
            ELEVATOR_STARTING_HEIGHT_METERS
    );
    static final ElevatorSim FOLLOWER_ELEVATOR_MOTOR = new ElevatorSim(
            FOLLOWER_ELEVATOR_MOTOR_GEARBOX,
            ELEVATOR_GEAR_RATIO,
            CARRIAGE_MASS_KG,
            DRUM_RADIUS_METERS,
            MIN_ELEVATOR_HEIGHT_METERS,
            MAX_ELEVATOR_HEIGHT_METERS,
            ELEVATOR_SIMULATE_GRAVITY,
            ELEVATOR_STARTING_HEIGHT_METERS
    );
}
