package frc.trigon.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Translation2d;

public class TurretConstants {

    private static final double
            HUB_X = 8.248,
            HUB_Y = 4.176;
    static final Translation2d HUB_POSE = new Translation2d(HUB_X, HUB_Y);
    static final double DEGREES_LIMIT = 200;
    static final double GEAR_RATIO = 100;
    static final double MOMENT_OF_INERTIA = 0.003;
}
