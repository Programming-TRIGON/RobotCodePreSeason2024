package frc.trigon.robot.subsystems.collector;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class CollectorConstants {
    private static final int MOTOR_ID = 1;
    private static final NeutralMode NEUTRAL_MODE_VALUE = NeutralMode.Coast;
    private static final boolean INVERTED = false;
    private static final int VOLTAGE_COMPENSATION_SATURATION = 12;
    static final TalonSRX MOTOR = new TalonSRX(MOTOR_ID);
    static final int MAX_CURRENT = 10;
    static final double MAX_CURRENT_TIME = 0.5;

    static {
        MOTOR.configFactoryDefault();
        MOTOR.configVoltageCompSaturation(VOLTAGE_COMPENSATION_SATURATION);
        MOTOR.enableVoltageCompensation(true);
        MOTOR.setNeutralMode(NEUTRAL_MODE_VALUE);
        MOTOR.setInverted(INVERTED);
    }

    public enum CollectorStates{
        COLLECT(-1),
        EJECT(1),
        HOLD(-0.2);

        final double power;

        CollectorStates(double power){
            this.power = power;
        }
    }

}
