package frc.trigon.robot.subsystems.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TurretConstants {
    private static final double
            HUB_X = 8.2296,
            HUB_Y = 0.5121;

    private static final Translation2d HUB_POSITION = new Translation2d(HUB_X, HUB_Y);
    private static final Rotation2d HUB_ROTATION = Rotation2d.fromRotations(0);

    static final double GEAR_RATIO = 100;
    static final double MOMENT_OF_INERTIA = 0.003;
    static final Pose2d HUB_POSE = new Pose2d(HUB_POSITION, HUB_ROTATION);

    private static final double
            P = 1,
            I = 0,
            D = 0;
    static final PIDController PID_CONTROLLER = new PIDController(P, I, D);
}
