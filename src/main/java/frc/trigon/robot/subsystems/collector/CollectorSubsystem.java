package frc.trigon.robot.subsystems.collector;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CollectorSubsystem extends SubsystemBase {
    private final static CollectorSubsystem INSTANCE = new CollectorSubsystem();
    private final TalonSRX motor = CollectorConstants.MOTOR;
    // motor.getSupplyCurrent

    public static CollectorSubsystem getInstance() {
        return INSTANCE;
    }

    private CollectorSubsystem() {
    }

    
}

