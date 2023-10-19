package frc.trigon.robot.subsystems.collector;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class CollectorConstants {

    private static final int MOTOR_ID = 1;
    private static final NeutralMode NEUTRAL_MODE_VALUE = NeutralMode.Brake;
    private static final boolean inverted = true;
    static final TalonSRX MOTOR = new TalonSRX(MOTOR_ID);
    static {
        MOTOR.setNeutralMode(NEUTRAL_MODE_VALUE);
        MOTOR.setInverted(inverted);
    }

    public enum CollectorStates{
        COLLECT(-1),
        EJECT(1),
        HOLD(-2);

        double power;
        CollectorStates(double power){
            this.power = power;
        }
    }

}
