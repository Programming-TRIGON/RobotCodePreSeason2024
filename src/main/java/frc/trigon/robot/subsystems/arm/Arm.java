package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private final static Arm INSTANCE = new Arm();
    private final CANSparkMax
            masterAngleMotor = ArmConstants.MASTER_ANGLE_MOTOR,
            followerAngleMotor = ArmConstants.FOLLOWER_ANGLE_MOTOR,
            masterElevatorMotor = ArmConstants.MASTER_ELEVATOR_MOTOR,
            followerElevatorMotor = ArmConstants.FOLLOWER_ELEVATOR_MOTOR;
    private final CANcoder angleEncoder = ArmConstants.ANGLE_ENCODER;
    private final TalonSRX elevatorEncoder = ArmConstants.ELEVATOR_ENCODER;
    private TrapezoidProfile
            angleMotorProfile = null,
            elevatorMotorProfile = null;
    private double
            lastAngleMotorProfileGenerationTime,
            lastElevatorMotorProfileGenerationTime;

    public static Arm getInstance() {
        return INSTANCE;
    }

    private Arm() {

    }

    /**
     * @return a command that sets the angle motor to the target angle
     */
    public Command getSetTargetArmAngleCommand(Rotation2d angle) {
        return new FunctionalCommand(
                () -> generateAngleMotorProfile(angle),
                this::setTargetAngleFromProfile,
                (interrupted) -> {
                },
                () -> false,
                this
        );
    }

    public Command getSetTargetElevatorPositionCommand(double targetPosition) {
        return new FunctionalCommand(
                () -> generateElevatorMotorProfile(targetPosition),
                this::setTargetPositionFromProfile,
                (interrupted) -> {
                },
                () -> false,
                this
        );
    }

    /**
     * @return a command that creates the profile for the angle motor
     */
    private void generateAngleMotorProfile(Rotation2d targetAngle) {
        angleMotorProfile = new TrapezoidProfile(
                ArmConstants.ANGLE_CONSTRAINTS,
                new TrapezoidProfile.State(targetAngle.getDegrees(), 0),
                new TrapezoidProfile.State(getAngleMotorPositionDegrees(), getAngleMotorVelocity())
        );
        lastAngleMotorProfileGenerationTime = Timer.getFPGATimestamp();
    }

    /**
     * @return a command that creates the profile for the elevator motor
     */
    private void generateElevatorMotorProfile(double targetPosition) {
        elevatorMotorProfile = new TrapezoidProfile(
                ArmConstants.ELEVATOR_CONSTRAINTS,
                new TrapezoidProfile.State(targetPosition * ArmConstants.ELEVATOR_MOTOR_RATIO, 0),
                new TrapezoidProfile.State(getElevatorMotorPositionDegrees(), getElevatorMotorVelocity())
        );
        lastElevatorMotorProfileGenerationTime = Timer.getFPGATimestamp();
    }

    /**
     * @return a command that moves the angle motor to the target angle
     */
    private void setTargetAngleFromProfile() {
        if (angleMotorProfile == null) {
            masterAngleMotor.stopMotor();
            return;
        }
        final TrapezoidProfile.State targetState = angleMotorProfile.calculate(getAngleMotorProfileTime());
        masterAngleMotor.getPIDController().setReference(targetState.position, CANSparkMax.ControlType.kPosition);
    }

    /**
     * @return a command that moves the elevator motor to the target position
     */
    private void setTargetPositionFromProfile() {
        if (elevatorMotorProfile == null) {
            masterElevatorMotor.stopMotor();
            return;
        }
        final TrapezoidProfile.State targetState = elevatorMotorProfile.calculate(getElevatorMotorProfileTime());
        masterElevatorMotor.getPIDController().setReference(targetState.position, CANSparkMax.ControlType.kPosition);
    }

    private double getAngleMotorProfileTime() {
        return Timer.getFPGATimestamp() - lastAngleMotorProfileGenerationTime;
    }

    private double getElevatorMotorProfileTime() {
        return Timer.getFPGATimestamp() - lastElevatorMotorProfileGenerationTime;
    }

    private double getAngleMotorPositionDegrees() {
        return angleEncoder.getAbsolutePosition().getValue();
    }

    private double getElevatorMotorPositionDegrees() {
        return elevatorEncoder.getSelectedSensorPosition();
    }

    private double getAngleMotorVelocity() {
        return angleEncoder.getVelocity().getValue();
    }

    private double getElevatorMotorVelocity() {
        return elevatorEncoder.getActiveTrajectoryVelocity();
    }
}

