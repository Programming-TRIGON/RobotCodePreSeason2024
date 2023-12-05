package frc.trigon.robot.subsystems.arm.simulationarm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class SimulationArmConstants {
    static final double VOLTAGE_COMPENSATION_SATURATION = 12;
    private static final int
            ANGLE_MOTOR_AMOUNT = 2,
            ELEVATOR_MOTOR_AMOUNT = 2;
    private static final DCMotor
            ANGLE_MOTOR_GEARBOX = DCMotor.getNEO(ANGLE_MOTOR_AMOUNT),
            ELEVATOR_MOTOR_GEARBOX = DCMotor.getNEO(ELEVATOR_MOTOR_AMOUNT);
    private static final double
            ANGLE_GEAR_RATIO = 12.4,
            ELEVATOR_GEAR_RATIO = 23952853.3;
    private static final double
            MINIMUM_ARM_LENGTH_METERS = 5,
            MAXIMUM_ARM_LENGTH_METERS = 20;
    private static final double ARM_MASS = 2;
    private static final Rotation2d
            MINIMUM_ANGLE = Rotation2d.fromDegrees(0),
            MAXIMUM_ANGLE = Rotation2d.fromDegrees(90);
    private static final boolean
            ANGLE_SIMULATE_GRAVITY = true,
            ELEVATOR_SIMULATE_GRAVITY = true;
    static final double ELEVATOR_METERS_PER_REVOLUTION = 0.3;
    private static final double ARM_DRUM_RADIUS_METERS = 3;

    static final SingleJointedArmSim ANGLE_MOTOR = new SingleJointedArmSim(
            ANGLE_MOTOR_GEARBOX,
            ANGLE_GEAR_RATIO,
            SingleJointedArmSim.estimateMOI(MAXIMUM_ARM_LENGTH_METERS, ARM_MASS),
            MAXIMUM_ARM_LENGTH_METERS,
            MINIMUM_ANGLE.getRadians(),
            MAXIMUM_ANGLE.getRadians(),
            ANGLE_SIMULATE_GRAVITY,
            MINIMUM_ANGLE.getRadians()
    );
    static final ElevatorSim ELEVATOR_MOTOR = new ElevatorSim(
            ELEVATOR_MOTOR_GEARBOX,
            ELEVATOR_GEAR_RATIO,
            ARM_MASS,
            ARM_DRUM_RADIUS_METERS,
            MINIMUM_ARM_LENGTH_METERS,
            MAXIMUM_ARM_LENGTH_METERS,
            ELEVATOR_SIMULATE_GRAVITY,
            MINIMUM_ARM_LENGTH_METERS
    );

    private static final double
            ANGLE_P = 0,
            ANGLE_I = 0,
            ANGLE_D = 0,
            ELEVATOR_P = 0,
            ELEVATOR_I = 0,
            ELEVATOR_D = 0;
    static final PIDController
            ANGLE_PID_CONTROLLER = new PIDController(ANGLE_P, ANGLE_I, ANGLE_D),
            ELEVATOR_PID_CONTROLLER = new PIDController(ELEVATOR_P, ELEVATOR_I, ELEVATOR_D);

    private static final double
            ANGLE_MOTOR_KS = 1,
            ANGLE_MOTOR_KG = 1,
            ANGLE_MOTOR_KV = 1,
            ANGLE_MOTOR_KA = 1,
            ELEVATOR_MOTOR_KS = 1,
            ELEVATOR_MOTOR_KG = 1,
            ELEVATOR_MOTOR_KV = 1,
            ELEVATOR_MOTOR_KA = 1;
    static final ArmFeedforward ANGLE_FEEDFORWARD = new ArmFeedforward(
            ANGLE_MOTOR_KS,
            ANGLE_MOTOR_KG,
            ANGLE_MOTOR_KV,
            ANGLE_MOTOR_KA
    );
    static final ElevatorFeedforward ELEVATOR_FEEDFORWARD = new ElevatorFeedforward(
            ELEVATOR_MOTOR_KS,
            ELEVATOR_MOTOR_KG,
            ELEVATOR_MOTOR_KV,
            ELEVATOR_MOTOR_KA
    );
}