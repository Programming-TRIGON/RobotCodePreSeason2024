package frc.trigon.robot.subsystems.collector;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;

public class CollectorConstants {
    static final int MAX_CURRENT = 10;
    static final double MAX_CURRENT_TIME = 0.5;

    public enum CollectorStates {
        COLLECT(-1),
        EJECT(1),
        HOLD(-0.2);

        final double power;

        CollectorStates(double power) {
            this.power = power;
        }
    }

}
