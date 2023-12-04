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
            ARM_MECHANISM_WIDTH = 1,
            ARM_MECHANISM_HEIGHT = 1;
    private static final double
            ARM_ROOT_WIDTH = 0,
            ARM_ROOT_LENGTH = 0;
    private static final double
            ARM_LIGAMENT_LENGTH = 0,
            ARM_LIGAMENT_ANGLE = 0;
    static final Mechanism2d ARM_MECHANISM = new Mechanism2d(ARM_MECHANISM_WIDTH, ARM_MECHANISM_HEIGHT);
    private static final MechanismRoot2d ARM_ROOT = ARM_MECHANISM.getRoot("ArmRoot", ARM_ROOT_WIDTH, ARM_ROOT_LENGTH);
    static final MechanismLigament2d ARM_LIGAMENT = ARM_ROOT.append(new MechanismLigament2d("ArmLigament", ARM_LIGAMENT_LENGTH, ARM_LIGAMENT_ANGLE));

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
