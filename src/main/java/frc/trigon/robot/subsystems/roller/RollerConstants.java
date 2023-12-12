package frc.trigon.robot.subsystems.roller;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class RollerConstants {
    static final int
            OPEN_POWER = 1,
            CLOSE_POWER = 1,
            COLLECTION_MOTOR_SPEED = 1;

    private static final double
            ROLLER_MECHANISM_WIDTH_METERS = 2,
            ROLLER_MECHANISM_HEIGHT_METERS = 1;
    private static final double
            ROLLER_MECHANISM_ROOT_X = 1,
            ROLLER_MECHANISM_ROOT_Y = 1;
    private static final double LIGAMENT_LINE_WIDTH = 10;
    static final Mechanism2d ROLLER_ANGLE_MECHANISM = new Mechanism2d(
            ROLLER_MECHANISM_WIDTH_METERS,
            ROLLER_MECHANISM_HEIGHT_METERS
    );
    static final MechanismRoot2d
            ROLLER_ANGLE_ROOT = ROLLER_ANGLE_MECHANISM.getRoot("ZRollerRoot", ROLLER_MECHANISM_ROOT_X, ROLLER_MECHANISM_ROOT_Y),
            TARGET_ROLLER_ANGLE_ROOT = ROLLER_ANGLE_MECHANISM.getRoot("TargetRollerAngleRoot", ROLLER_MECHANISM_ROOT_X, ROLLER_MECHANISM_ROOT_Y);
    static final MechanismLigament2d
            ROLLER_ANGLE_LIGAMENT = ROLLER_ANGLE_ROOT.append(new MechanismLigament2d("RollerLigament", 0, 0, LIGAMENT_LINE_WIDTH, new Color8Bit(Color.kBlue))),
            TARGET_ROLLER_ANGLE_LIGAMENT = TARGET_ROLLER_ANGLE_ROOT.append(new MechanismLigament2d("TargetRollerAngleLigament", 0, 0, LIGAMENT_LINE_WIDTH, new Color8Bit(Color.kGray)));
}