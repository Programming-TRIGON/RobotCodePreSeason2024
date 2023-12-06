package frc.trigon.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
    private final static Turret INSTANCE = new Turret();

    public static Turret getInstance() {
        return INSTANCE;
    }

    private Turret() {
    }
}
