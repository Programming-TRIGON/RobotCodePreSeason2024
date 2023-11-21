package frc.trigon.robot.subsystems.collector.kablamacollector;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class KablamaCollectorConstants {
    private static final int MOTOR_ID = 1;
    private static final NeutralMode NEUTRAL_MODE_VALUE = NeutralMode.Coast;
    private static final boolean INVERTED = false;
    private static final int VOLTAGE_COMPENSATION_SATURATION = 12;
    static final TalonSRX MOTOR = new TalonSRX(MOTOR_ID);

    static {
        MOTOR.configFactoryDefault();
        MOTOR.configVoltageCompSaturation(VOLTAGE_COMPENSATION_SATURATION);
        MOTOR.enableVoltageCompensation(true);
        MOTOR.setNeutralMode(NEUTRAL_MODE_VALUE);
        MOTOR.setInverted(INVERTED);
    }
}
