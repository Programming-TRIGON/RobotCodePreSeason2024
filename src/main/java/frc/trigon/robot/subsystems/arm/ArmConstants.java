package frc.trigon.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

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
            MECHANISM2D_WIDTH = 1,
            MECHANISM2D_HEIGHT = 1;
    private static final double
            MECHANISM2D_ROOT_X = 0,
            MECHANISM2D_ROOT_Y = 0;
    private static final double
            MECHANISM2D_LIGAMENT_LENGTH = 0,
            MECHANISM2D_LIGAMENT_ANGLE = 0;
    static final Mechanism2d ARM_MECHANISM = new Mechanism2d(MECHANISM2D_WIDTH, MECHANISM2D_HEIGHT);
    static final MechanismRoot2d ARM_ROOT = ARM_MECHANISM.getRoot("Arm", MECHANISM2D_ROOT_X, MECHANISM2D_ROOT_Y);
    static final MechanismLigament2d ARM_LIGAMENT = ARM_ROOT.append(new MechanismLigament2d("Arm", MECHANISM2D_LIGAMENT_LENGTH, MECHANISM2D_LIGAMENT_ANGLE));

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
