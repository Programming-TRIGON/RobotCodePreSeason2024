package frc.trigon.robot.subsystems.collector;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class CollectorConstants {

    private static final int MOTOR_ID = 1;
    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    private static final InvertedValue INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    static final TalonSRX MOTOR = new TalonSRX(MOTOR_ID);
    static {
        TalonSRXConfiguration config = new TalonSRXConfiguration();

        MOTOR.getAllConfigs(config);
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
