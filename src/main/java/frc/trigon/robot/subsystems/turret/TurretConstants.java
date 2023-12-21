package frc.trigon.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class TurretConstants {
    static final double DEGREES_LIMIT = 200;

    private static final double
            HUB_X = 8.248,
            HUB_Y = 4.176;
    static final Translation2d HUB_POSITION = new Translation2d(HUB_X, HUB_Y);

    private static final double
            TURRET_MECHANISM_WIDTH = 10,
            TURRET_MECHANISM_HEIGHT = 10;
    private static final double
            TURRET_ROOT_X = 1,
            TURRET_ROOT_Y = 1;
    private static final double
            TURRET_ROOT_LENGTH = 1,
            TURRET_ROOT_ANGLE = 0;
    private static final double MECHANISM_LINE_WIDTH = 10;
    static final Mechanism2d TURRET_MECHANISM = new Mechanism2d(TURRET_MECHANISM_WIDTH, TURRET_MECHANISM_HEIGHT);
    static final MechanismRoot2d TURRET_ROOT = TURRET_MECHANISM.getRoot("TURRET_ROOT", TURRET_ROOT_X, TURRET_ROOT_Y);
    static final MechanismLigament2d TURRET_LIGAMENT = TURRET_ROOT.append(new MechanismLigament2d("TURRET_LIGAMENT", TURRET_ROOT_LENGTH, TURRET_ROOT_ANGLE, MECHANISM_LINE_WIDTH, new Color8Bit(Color.kBlue)));
}
