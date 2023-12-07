package frc.trigon.robot.subsystems.turret;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
    private final static Turret INSTANCE = new Turret();
    private final TalonFX motor = TurretConstants.MOTOR;
    private final CANcoder encoder = TurretConstants.ENCODER;
    private final Pose2d pose = TurretConstants.POSE;


    public static Turret getInstance() {
        return INSTANCE;
    }

    private Turret() {
    }
}
