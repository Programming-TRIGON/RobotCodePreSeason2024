package frc.trigon.robot.subsystems.collector;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class CollectorConstants {
    static final int MAX_CURRENT = 10;
    static final double MAX_CURRENT_TIME = 0.5;

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
