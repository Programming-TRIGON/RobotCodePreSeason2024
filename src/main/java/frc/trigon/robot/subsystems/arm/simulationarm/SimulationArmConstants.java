package frc.trigon.robot.subsystems.arm.simulationarm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.trigon.robot.subsystems.arm.ArmConstants;

public class SimulationArmConstants {
    private static final int
            ANGLE_MOTOR_AMOUNT = 2,
            ELEVATOR_MOTOR_AMOUNT = 2;
    private static final DCMotor
            ANGLE_MOTOR_GEARBOX = DCMotor.getNEO(ANGLE_MOTOR_AMOUNT),
            ELEVATOR_MOTOR_GEARBOX = DCMotor.getNEO(ELEVATOR_MOTOR_AMOUNT);
    private static final double
            ANGLE_GEAR_RATIO = 95.2,
            ELEVATOR_GEAR_RATIO = 13.29;
    private static final Rotation2d
            MINIMUM_ANGLE = Rotation2d.fromDegrees(-45),
            MAXIMUM_ANGLE = Rotation2d.fromDegrees(180);
    private static final boolean
            ANGLE_SIMULATE_GRAVITY = true,
            ELEVATOR_SIMULATE_GRAVITY = true;
    private static final double ARM_MASS = 12;
    private static final double DRUM_RADIUS_METERS = 0.1256 / 2;
    private static final double FULLY_OPENED_ARM_LENGTH_METERS = 1.7;
    static final SingleJointedArmSim ANGLE_MOTOR = new SingleJointedArmSim(
            ANGLE_MOTOR_GEARBOX,
            ANGLE_GEAR_RATIO,
            SingleJointedArmSim.estimateMOI(ArmConstants.RETRACTED_ARM_LENGTH_METERS, ARM_MASS),
            ArmConstants.RETRACTED_ARM_LENGTH_METERS,
            MINIMUM_ANGLE.getRadians(),
            MAXIMUM_ANGLE.getRadians(),
            ANGLE_SIMULATE_GRAVITY,
            MINIMUM_ANGLE.getRadians()
    );
    static final ElevatorSim ELEVATOR_MOTOR = new ElevatorSim(
            ELEVATOR_MOTOR_GEARBOX,
            ELEVATOR_GEAR_RATIO,
            ARM_MASS,
            DRUM_RADIUS_METERS,
            ArmConstants.RETRACTED_ARM_LENGTH_METERS,
            FULLY_OPENED_ARM_LENGTH_METERS,
            ELEVATOR_SIMULATE_GRAVITY,
            ArmConstants.RETRACTED_ARM_LENGTH_METERS
    );

    private static final double
            ANGLE_P = 1.5,
            ANGLE_I = 0,
            ANGLE_D = 0,
            ELEVATOR_P = 5,
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
