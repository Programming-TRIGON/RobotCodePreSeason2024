package frc.trigon.robot.subsystems.collector;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class CollectorConstants {
    private static final int MOTOR_ID = 1;
    private static final NeutralMode NEUTRAL_MODE_VALUE = NeutralMode.Brake;
    private static final boolean INVERTED = true;
    private static final int VOLTAGE_COMP_SATURATION = 11;
    static final TalonSRX MOTOR = new TalonSRX(MOTOR_ID);

    static {
        MOTOR.configFactoryDefault();
        MOTOR.configVoltageCompSaturation(VOLTAGE_COMP_SATURATION);
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
