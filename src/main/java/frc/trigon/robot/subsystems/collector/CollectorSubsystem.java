package frc.trigon.robot.subsystems.collector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class CollectorSubsystem extends SubsystemBase {
    private final static CollectorSubsystem INSTANCE = new CollectorSubsystem();

    public static CollectorSubsystem getInstance() {
        return INSTANCE;
    }

    private CollectorSubsystem() {
    }
}

