package frc.trigon.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.AutoLogOutput;

public class TurretConstants {
    private static final double
            MAX_MOTOR_VELOCITY = 100,
            MAX_MOTOR_ACCELERATION = 2;
    static final TrapezoidProfile.Constraints MOTOR_CONSTRAINTS = new TrapezoidProfile.Constraints(MAX_MOTOR_VELOCITY, MAX_MOTOR_ACCELERATION);
    static final Translation2d HUB_POSITION = new Translation2d(8.2296, 0.5121);

    private static final double
            TURRET_MECHANISM_WIDTH = 3,
            TURRET_MECHANISM_HEIGHT = 3,
            LIGAMENT_LINE_WIDTH = 10;
    private static final double
            TURRET_MECHANISM_ROOT_X = 1,
            TURRET_MECHANISM_ROOT_Y = 1;
    @AutoLogOutput(key = "Turret/TurretMechanism")
    static final Mechanism2d TURRET_MECHANISM = new Mechanism2d(
            TURRET_MECHANISM_WIDTH,
            TURRET_MECHANISM_HEIGHT
    );
    private static final MechanismRoot2d
            TURRET_MECHANISM_ROOT = TURRET_MECHANISM.getRoot("ZTurretRoot", TURRET_MECHANISM_ROOT_X, TURRET_MECHANISM_ROOT_Y),
            TARGET_TURRET_POSITION_ROOT = TURRET_MECHANISM.getRoot("TargetTurretPosition", TURRET_MECHANISM_ROOT_X, TURRET_MECHANISM_ROOT_Y);
    static final MechanismLigament2d
            TURRET_LIGAMENT = TURRET_MECHANISM_ROOT.append(new MechanismLigament2d("TurretLigament", 0, 0, LIGAMENT_LINE_WIDTH, new Color8Bit(Color.kBlue))),
            TARGET_TURRET_POSITION_LIGAMENT = TARGET_TURRET_POSITION_ROOT.append(new MechanismLigament2d("TargetTurretPositionLigament", 0, 0, LIGAMENT_LINE_WIDTH, new Color8Bit(Color.kGray)));
}