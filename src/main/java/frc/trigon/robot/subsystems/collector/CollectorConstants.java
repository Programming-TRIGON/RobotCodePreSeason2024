package frc.trigon.robot.subsystems.collector;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
public class CollectorConstants {
    private static final int MOTOR_ID = 1;
    private static final NeutralMode NEUTRAL_MODE_VALUE = NeutralMode.Brake;
    private static final boolean INVERTED = true;
    private static final int VOLTAGE_COMP_SATURATION = 11;
    private static final boolean ENABLE_VOLTAGE_COMPENSATION = true;
    static final TalonSRX MOTOR = new TalonSRX(MOTOR_ID);

    static {
        MOTOR.configVoltageCompSaturation(VOLTAGE_COMP_SATURATION);
        MOTOR.enableVoltageCompensation(ENABLE_VOLTAGE_COMPENSATION);
        MOTOR.configFactoryDefault();
        MOTOR.setNeutralMode(NEUTRAL_MODE_VALUE);
        MOTOR.setInverted(INVERTED);
    }

    public enum CollectorStates{
        COLLECT(-1),
        EJECT(1),
        HOLD(-0.2);

        double power;

        CollectorStates(double power){
            this.power = power;
        }
    }

}
