package frc.trigon.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    private final static ArmSubsystem INSTANCE = new ArmSubsystem();

    @SuppressWarnings("WeakerAccess")
    public static ArmSubsystem getInstance() {
        return INSTANCE;
    }

    private ArmSubsystem() {
    }
}

