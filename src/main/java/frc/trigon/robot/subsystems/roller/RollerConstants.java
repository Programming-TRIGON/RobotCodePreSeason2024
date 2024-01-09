package frc.trigon.robot.subsystems.roller;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.AutoLogOutput;

public class RollerConstants {
    static final double
            OPEN_POWER = 0.3,
            CLOSE_POWER = -0.3,
            COLLECTION_MOTOR_SPEED = 1;

    public static final double ROLLER_LENGTH_METERS = 0.45;
    private static final double
            ROLLER_MECHANISM_ROOT_X = 0.1,
            ROLLER_MECHANISM_ROOT_Y = 0.1;
    private static final double LIGAMENT_LINE_WIDTH = 10;
    @AutoLogOutput(key = "Roller/RollerMechanism")
    static final Mechanism2d ROLLER_ANGLE_MECHANISM = new Mechanism2d(
            ROLLER_LENGTH_METERS,
            ROLLER_LENGTH_METERS
    );
    static final MechanismRoot2d ROLLER_ANGLE_ROOT = ROLLER_ANGLE_MECHANISM.getRoot("ZRollerRoot", ROLLER_MECHANISM_ROOT_X, ROLLER_MECHANISM_ROOT_Y);
    static final MechanismLigament2d ROLLER_ANGLE_LIGAMENT = ROLLER_ANGLE_ROOT.append(new MechanismLigament2d("RollerLigament", ROLLER_LENGTH_METERS, 0, LIGAMENT_LINE_WIDTH, new Color8Bit(Color.kBlue)));
}