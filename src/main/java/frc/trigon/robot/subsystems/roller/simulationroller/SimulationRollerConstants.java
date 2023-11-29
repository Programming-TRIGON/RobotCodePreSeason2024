package frc.trigon.robot.subsystems.roller.simulationroller;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.trigon.robot.subsystems.roller.RollerConstants;

public class SimulationRollerConstants extends RollerConstants {
    private static int
            NUMBER_ANGLE_MOTORS = 1,
            NUMBER_COLLECTOR_MOTORS = 1;
    private static final double ROLLER_MASS = 10;
    private static final boolean ANGLE_SIMULATE_GRAVITY = true;
    private static final Rotation2d
            MIN_ANGLE = Rotation2d.fromDegrees(0),
            MAX_ANGLE = Rotation2d.fromDegrees(90),
            STARTING_ANGLE = Rotation2d.fromDegrees(90);
    private static final double
            ANGLE_GEAR_RATIO = 8.4,
            COLLECTOR_GEAR_RATIO = 23452234.1;
    private static final double COLLECTOR_MOMENT_OF_INERTIA = 100000000;
    private static DCMotor
            ANGLE_MOTOR_GEARBOX = DCMotor.getNEO(NUMBER_ANGLE_MOTORS),
            COLLECTOR_MOTOR_GEARBOX = DCMotor.getNEO(NUMBER_COLLECTOR_MOTORS);
    private static SingleJointedArmSim ANGLE_MOTOR_SIM = new SingleJointedArmSim(
            ANGLE_MOTOR_GEARBOX,
            ANGLE_GEAR_RATIO,
            SingleJointedArmSim.estimateMOI(RollerConstants.ROLLER_LENGTH, ROLLER_MASS),
            RollerConstants.ROLLER_LENGTH,
            MIN_ANGLE.getRadians(),
            MAX_ANGLE.getRadians(),
            ANGLE_SIMULATE_GRAVITY,
            STARTING_ANGLE.getRadians()
    );
    private static DCMotorSim COLLECTOR_MOTOR_SIM = new DCMotorSim(
            COLLECTOR_MOTOR_GEARBOX,
            COLLECTOR_GEAR_RATIO,
            COLLECTOR_MOMENT_OF_INERTIA
    );
}
