package frc.trigon.robot.subsystems.collector.simulationcollector;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimulationCollectorIOConstants {
    private static final double
            NOMINAL_VOLTAGE_VOLTS = 1,
            STALL_TORQUE_NEWTON_METERS = 1,
            STALL_CURRENT_AMPS = 1,
            FREE_CURRENT_AMPS = 1,
            FREE_SPEED_RAD_PER_SEC = 1;
    private static final int NUM_MOTORS = 1;
    private static final double
            JKG_METERS_SQUARED = 1,
            G = 1,
            GEARING = 1;
    static final DCMotor DC_MOTOR =
            new DCMotor(
                    NOMINAL_VOLTAGE_VOLTS,
                    STALL_TORQUE_NEWTON_METERS,
                    STALL_CURRENT_AMPS,
                    FREE_CURRENT_AMPS,
                    FREE_SPEED_RAD_PER_SEC,
                    NUM_MOTORS
            );
    static final DCMotorSim DC_MOTOR_SIM =
            new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(DC_MOTOR, JKG_METERS_SQUARED, G),
                    DC_MOTOR,
                    GEARING
            );
}
