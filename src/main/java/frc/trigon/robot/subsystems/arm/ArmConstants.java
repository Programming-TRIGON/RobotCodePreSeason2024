package frc.trigon.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ArmConstants {

    public static final double RETRACTED_ARM_LENGTH_METERS = 0.64;
    private static final double
            MAX_ANGLE_VELOCITY = 150,
            MAX_ANGLE_ACCELERATION = 190,
            MAX_ELEVATOR_VELOCITY = 2,
            MAX_ELEVATOR_ACCELERATION = 3;
    static final TrapezoidProfile.Constraints
            ANGLE_CONSTRAINTS = new TrapezoidProfile.Constraints(
            MAX_ANGLE_VELOCITY, MAX_ANGLE_ACCELERATION
    ),
            ELEVATOR_CONSTRAINTS = new TrapezoidProfile.Constraints(
                    MAX_ELEVATOR_VELOCITY, MAX_ELEVATOR_ACCELERATION
            );

    private static final double
            ARM_MECHANISM_WIDTH = 4,
            ARM_MECHANISM_HEIGHT = 4;
    private static final double
            ARM_ROOT_X = 1,
            ARM_ROOT_Y = 1;
    private static final double MECHANISM_LINE_WIDTH = 10;

    static final Mechanism2d ARM_MECHANISM = new Mechanism2d(ARM_MECHANISM_WIDTH, ARM_MECHANISM_HEIGHT);
    private static final MechanismRoot2d
            ARM_ROOT = ARM_MECHANISM.getRoot("Z_ARM_ROOT", ARM_ROOT_X, ARM_ROOT_Y),
            TARGET_POSITION_ROOT = ARM_MECHANISM.getRoot("TargetPositionRoot", ARM_ROOT_X, ARM_ROOT_Y);
    static final MechanismLigament2d
            ARM_LIGAMENT = ARM_ROOT.append(new MechanismLigament2d("ArmLigament", 0, 0, MECHANISM_LINE_WIDTH, new Color8Bit(Color.kBlue))),
            TARGET_POSITION_LIGAMENT = TARGET_POSITION_ROOT.append(new MechanismLigament2d("TargetPositionLigament", 0, 0, MECHANISM_LINE_WIDTH, new Color8Bit(Color.kGray)));
    static final double
            ANGLE_TOLERANCE_DEGREES = 1,
            ELEVATOR_TOLERANCE_METERS = 0.01;

    public enum ArmState {
        DEFAULT_STATE(Rotation2d.fromDegrees(50), 0.9),
        CONE_COLLECTION(Rotation2d.fromDegrees(15), 1),
        HIGH_CONE(Rotation2d.fromDegrees(85), 1.6),
        LOW_CONE(Rotation2d.fromDegrees(25), 0.4);

        final Rotation2d angle;
        final double elevatorPositionMeters;

        ArmState(Rotation2d angle, double elevatorPositionMeters) {
            this.angle = angle;
            this.elevatorPositionMeters = elevatorPositionMeters;
        }
    }
}
