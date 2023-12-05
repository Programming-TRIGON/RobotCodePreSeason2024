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
            MAX_ANGLE_ACCELERATION = 100,
            MAX_ELEVATOR_VELOCITY = 100,
            MAX_ELEVATOR_ACCELERATION = 100;
    static final TrapezoidProfile.Constraints
            ANGLE_CONSTRAINTS = new TrapezoidProfile.Constraints(
            MAX_ANGLE_VELOCITY, MAX_ANGLE_ACCELERATION
    ),
            ELEVATOR_CONSTRAINTS = new TrapezoidProfile.Constraints(
                    MAX_ELEVATOR_VELOCITY, MAX_ELEVATOR_ACCELERATION
            );

    private static final double
            ARM_MECHANISM_WIDTH = 2,
            ARM_MECHANISM_HEIGHT = 2;
    private static final double
            ARM_ROOT_WIDTH = 1,
            ARM_ROOT_LENGTH = 1;
    private static final double
            ARM_LIGAMENT_LENGTH = 2,
            ARM_LIGAMENT_ANGLE = 2;
    private static final double ARM_MECHANISM_LENGTH = 1;
    static final Mechanism2d ARM_MECHANISM = new Mechanism2d(ARM_MECHANISM_WIDTH, ARM_MECHANISM_HEIGHT);
    private static final MechanismRoot2d
            ARM_ROOT = ARM_MECHANISM.getRoot("ArmRoot", ARM_ROOT_WIDTH, ARM_ROOT_LENGTH),
            TARGET_POSITION_ROOT = ARM_MECHANISM.getRoot("TargetPositionRoot", ARM_ROOT_WIDTH, ARM_ROOT_LENGTH);
    static final MechanismLigament2d
            ARM_LIGAMENT = ARM_ROOT.append(new MechanismLigament2d("ArmLigament", ARM_LIGAMENT_LENGTH, ARM_LIGAMENT_ANGLE, ARM_MECHANISM_LENGTH, new Color8Bit(Color.kBlue))),
            TARGET_POSITION_LIGAMENT = TARGET_POSITION_ROOT.append(new MechanismLigament2d("TargetPositionLigament", ARM_LIGAMENT_LENGTH, ARM_LIGAMENT_ANGLE, ARM_MECHANISM_LENGTH, new Color8Bit(Color.kGray)));
    static final double
            ANGLE_TOLERANCE = 1,
            ELEVATOR_TOLERANCE = 1;
    public static final double RETRACTED_ARM_LENGTH_METERS = 1;
    public static final double THEORETICAL_METERS_PER_REVOLUTION = 0.1256;

    public enum ArmState {
        FIRST_STATE(Rotation2d.fromDegrees(100), 7),
        SECOND_STATE(Rotation2d.fromDegrees(70), 5);

        final Rotation2d angle;
        final double elevatorPosition;

        ArmState(Rotation2d angle, double elevatorPosition) {
            this.angle = angle;
            this.elevatorPosition = elevatorPosition;
        }
    }
}
