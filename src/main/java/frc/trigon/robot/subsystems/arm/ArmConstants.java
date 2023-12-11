package frc.trigon.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ArmConstants {
    private static final double
            MAX_ANGLE_VELOCITY = 150,
            MAX_ELEVATOR_VELOCITY = 2,
            MAX_ANGLE_ACCELERATION = 190,
            MAX_ELEVATOR_ACCELERATION = 3;

    static final TrapezoidProfile.Constraints
            ANGLE_CONSTRAINTS = new TrapezoidProfile.Constraints(MAX_ANGLE_VELOCITY, MAX_ANGLE_ACCELERATION),
            ELEVATOR_CONSTRAINTS = new TrapezoidProfile.Constraints(MAX_ELEVATOR_VELOCITY, MAX_ELEVATOR_ACCELERATION);
    static final double
            ANGLE_TOLERANCE_DEGREES = 0.5,
            ELEVATOR_TOLERANCE_METERS = 0.02;
    public static final double
            ARM_MECHANISM_WIDTH_METERS = 3,
            ARM_MECHANISM_HEIGHT_METERS = 3,
            ARM_LIGAMENT_LENGTH_METERS = 0.64,
            ARM_LIGAMENT_ANGLE_DEGREES = 0,
            RETRACTED_ARM_LENGTH_METERS = 0.64;
    private static final double
            ARM_MECHANISM_ROOT_X = 1,
            ARM_MECHANISM_ROOT_Y = 1;
    static final Mechanism2d ARM_MECHANISM = new Mechanism2d(
            ARM_MECHANISM_WIDTH_METERS,
            ARM_MECHANISM_HEIGHT_METERS
    );
    private static final MechanismRoot2d
            ARM_ROOT = ARM_MECHANISM.getRoot("ArmRoot", ARM_MECHANISM_ROOT_X, ARM_MECHANISM_ROOT_Y),
            TARGET_ARM_POSITION_ROOT = ARM_MECHANISM.getRoot("TargetArmPositionRoot", ARM_MECHANISM_ROOT_X, ARM_MECHANISM_ROOT_Y);
    static final MechanismLigament2d
            ARM_LIGAMENT = ARM_ROOT.append(new MechanismLigament2d("ArmLigament", ARM_LIGAMENT_LENGTH_METERS, ARM_LIGAMENT_ANGLE_DEGREES, 5, new Color8Bit(Color.kCyan))),
            TARGET_ARM_POSITION_LIGAMENT = TARGET_ARM_POSITION_ROOT.append(new MechanismLigament2d("TargetArmPositionLigament", ARM_LIGAMENT_LENGTH_METERS, ARM_LIGAMENT_ANGLE_DEGREES, 3, new Color8Bit(Color.kWhite)));

    public enum ArmState {
        DEFAULT(Rotation2d.fromDegrees(0), 0),
        TAKE_HIGH_CONE(Rotation2d.fromDegrees(60), 1),
        TAKE_GROUND_CONE(Rotation2d.fromDegrees(5), 0.8),
        PLACE_LOW_CONE(Rotation2d.fromDegrees(20), 0.4),
        PLACE_MEDIUM_CONE(Rotation2d.fromDegrees(45), 0.6),
        PLACE_HIGH_CONE(Rotation2d.fromDegrees(90), 1.1);

        final Rotation2d angle;
        final double elevatorPositionMeters;

        ArmState(Rotation2d angle, double elevatorPositionMeters) {
            this.angle = angle;
            this.elevatorPositionMeters = elevatorPositionMeters;
        }
    }
}
