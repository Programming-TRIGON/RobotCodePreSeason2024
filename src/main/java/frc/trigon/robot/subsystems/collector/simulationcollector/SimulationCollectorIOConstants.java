package frc.trigon.robot.subsystems.collector.simulationcollector;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.system.plant.DCMotor;

public class SimulationCollectorIOConstants {
    private static final int MOTOR_ID = 1;
    private static final NeutralMode NEUTRAL_MODE_VALUE = NeutralMode.Coast;
    private static final boolean INVERTED = false;
    private static final int VOLTAGE_COMPENSATION_SATURATION = 12;
    private static final double
            nominalVoltageVolts = 1,
            stallTorqueNewtonMeters = 1,
            stallCurrentAmps = 1,
            freeCurrentAmps = 1,
            freeSpeedRadPerSec = 1;
    private static final int numMotors = 1;
    static final TalonSRX MOTOR = new TalonSRX(MOTOR_ID);
    static final DCMotor DC_MOTOR = new DCMotor(nominalVoltageVolts, stallTorqueNewtonMeters, stallCurrentAmps, freeCurrentAmps, freeSpeedRadPerSec, numMotors);

    static {
        MOTOR.configFactoryDefault();
        MOTOR.configVoltageCompSaturation(VOLTAGE_COMPENSATION_SATURATION);
        MOTOR.enableVoltageCompensation(true);
        MOTOR.setNeutralMode(NEUTRAL_MODE_VALUE);
        MOTOR.setInverted(INVERTED);
    }
}
