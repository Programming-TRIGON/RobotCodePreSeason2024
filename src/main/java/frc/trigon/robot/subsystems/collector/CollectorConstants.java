package frc.trigon.robot.subsystems.collector;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;

public class CollectorConstants {
    static final int MAX_CURRENT = 10;
    static final double MAX_CURRENT_TIME = 0.5;
    private static final double
            MECHANISM2D_WIDTH = 1,
            MECHANISM2D_HEIGHT = 1;
    static final Mechanism2d COLLECTOR_MECHANISM = new Mechanism2d(MECHANISM2D_WIDTH, MECHANISM2D_HEIGHT);

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
