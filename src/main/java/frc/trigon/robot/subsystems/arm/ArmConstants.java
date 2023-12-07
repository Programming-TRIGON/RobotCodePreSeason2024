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
            MAX_ANGLE_VELOCITY = 100,
            MAX_ELEVATOR_VELOCITY = 100,
            MAX_ANGLE_ACCELERATION = 30,
            MAX_ELEVATOR_ACCELERATION = 30;

    static final TrapezoidProfile.Constraints
            ANGLE_CONSTRAINTS = new TrapezoidProfile.Constraints(MAX_ANGLE_VELOCITY, MAX_ANGLE_ACCELERATION),
            ELEVATOR_CONSTRAINTS = new TrapezoidProfile.Constraints(MAX_ELEVATOR_VELOCITY, MAX_ELEVATOR_ACCELERATION);
    static final double
            ANGLE_TOLERANCE = 0.5,
            ELEVATOR_TOLERANCE = 0.7;
    static final double
            ARM_WIDTH_METERS = 20,
            ARM_HEIGHT_METERS = 20;

    private static final double
            ARM_MECHANISM_ROOT_X = 10,
            ARM_MECHANISM_ROOT_Y = 10;
    private static final double ARM_LENGTH_METERS = 3;
    private static final Rotation2d ARM_ANGLE = Rotation2d.fromDegrees(2);
    static final Mechanism2d ARM_MECHANISM = new Mechanism2d(
            ARM_WIDTH_METERS,
            ARM_HEIGHT_METERS
    );
    private static final MechanismRoot2d
            ARM_ROOT = ARM_MECHANISM.getRoot("ArmRoot", ARM_MECHANISM_ROOT_X, ARM_MECHANISM_ROOT_Y),
            TARGET_ARM_POSITION_ROOT = ARM_MECHANISM.getRoot("TargetArmPositionRoot", ARM_MECHANISM_ROOT_X, ARM_MECHANISM_ROOT_Y);
    static final MechanismLigament2d
            ARM_LIGAMENT = ARM_ROOT.append(new MechanismLigament2d("ArmLigament", ARM_LENGTH_METERS, ARM_ANGLE.getDegrees(), 5, new Color8Bit(Color.kCyan))),
            TARGET_ARM_POSITION_LIGAMENT = TARGET_ARM_POSITION_ROOT.append(new MechanismLigament2d("TargetArmPositionLigament", ARM_LENGTH_METERS, ARM_ANGLE.getDegrees(), 3, new Color8Bit(Color.kWhite)));

    public enum ArmState {
        DEFAULT(Rotation2d.fromDegrees(0), 0),
        TAKE_HIGH_CONE(Rotation2d.fromDegrees(60), 10),
        TAKE_GROUND_CONE(Rotation2d.fromDegrees(0), 0),
        PLACE_LOW_CONE(Rotation2d.fromDegrees(20), 3),
        PLACE_MEDIUM_CONE(Rotation2d.fromDegrees(50), 7),
        PLACE_HIGH_CONE(Rotation2d.fromDegrees(70), 10);

        final Rotation2d angle;
        final double elevatorPositionMeters;

        ArmState(Rotation2d angle, double elevatorPosition) {
            this.angle = angle;
            this.elevatorPositionMeters = elevatorPosition;
        }
    }
}
