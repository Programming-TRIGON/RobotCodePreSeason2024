package frc.trigon.robot.subsystems.roller.simulationroller;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class SimulationRollerConstants {
    private static int
            ANGLE_MOTOR_AMOUNT = 1,
            COLLECTOR_MOTOR_AMOUNT = 1;
    private static final double ROLLER_MASS = 10;
    private static final boolean ANGLE_SIMULATE_GRAVITY = true;

    private static final double ROLLER_LENGTH = 200;
    private static final Rotation2d
            MIN_ANGLE = Rotation2d.fromDegrees(0),
            MAX_ANGLE = Rotation2d.fromDegrees(90);
    private static final double
            ANGLE_GEAR_RATIO = 8.4,
            COLLECTOR_GEAR_RATIO = 23452234.1;
    private static final double COLLECTOR_MOMENT_OF_INERTIA = 0.0003;
    private static DCMotor
            ANGLE_MOTOR_GEARBOX = DCMotor.getNEO(ANGLE_MOTOR_AMOUNT),
            COLLECTOR_MOTOR_GEARBOX = DCMotor.getNEO(COLLECTOR_MOTOR_AMOUNT);
    static SingleJointedArmSim ANGLE_MOTOR = new SingleJointedArmSim(
            ANGLE_MOTOR_GEARBOX,
            ANGLE_GEAR_RATIO,
            SingleJointedArmSim.estimateMOI(ROLLER_LENGTH, ROLLER_MASS),
            ROLLER_LENGTH,
            MIN_ANGLE.getRadians(),
            MAX_ANGLE.getRadians(),
            ANGLE_SIMULATE_GRAVITY,
            MAX_ANGLE.getRadians()
    );
    static DCMotorSim COLLECTOR_MOTOR = new DCMotorSim(
            COLLECTOR_MOTOR_GEARBOX,
            COLLECTOR_GEAR_RATIO,
            COLLECTOR_MOMENT_OF_INERTIA
    );
}
