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

import static frc.trigon.robot.utilities.Conversions.magTicksToRevolutions;

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

    public Command getSetTargetElevatorPositionCommand(double targetElevatorPosition) {
        return new FunctionalCommand(
                () -> generateElevatorMotorProfile(targetElevatorPosition),
                this::setTargetPositionFromProfile,
                (interrupted) -> {
                },
                () -> false,
                this
        );
    }

    private void generateAngleMotorProfile(Rotation2d targetAngle) {
        angleMotorProfile = new TrapezoidProfile(
                ArmConstants.ANGLE_CONSTRAINTS,
                new TrapezoidProfile.State(targetAngle.getDegrees(), 0),
                new TrapezoidProfile.State(getAngleMotorPositionDegrees().getDegrees(), getAngleMotorVelocityInDegreesPerSeconds())
        );
        lastAngleMotorProfileGenerationTime = Timer.getFPGATimestamp();
    }

    private void generateElevatorMotorProfile(double targetElevatorPosition) {
        elevatorMotorProfile = new TrapezoidProfile(
                ArmConstants.ELEVATOR_CONSTRAINTS,
                new TrapezoidProfile.State(targetElevatorPosition, 0),
                new TrapezoidProfile.State(getElevatorMotorPositionRevolutions(), getElevatorMotorVelocityInRevolutionsPerSecond())
        );
        lastElevatorMotorProfileGenerationTime = Timer.getFPGATimestamp();
    }

    private void setTargetAngleFromProfile() {
        if (angleMotorProfile == null) {
            stopAngleMotors();
            return;
        }
        final TrapezoidProfile.State targetState = angleMotorProfile.calculate(getAngleMotorProfileTime());
        ArmConstants.MASTER_ANGLE_MOTOR.getPIDController().setP(ArmConstants.ANGLE_PID_CONTROLLER.getP());
        ArmConstants.MASTER_ANGLE_MOTOR.getPIDController().setI(ArmConstants.ANGLE_PID_CONTROLLER.getI());
        ArmConstants.MASTER_ANGLE_MOTOR.getPIDController().setD(ArmConstants.ANGLE_PID_CONTROLLER.getD());
    }

    private void setTargetPositionFromProfile() {
        if (elevatorMotorProfile == null) {
            stopElevatorMotors();
            return;
        }
        final TrapezoidProfile.State targetState = elevatorMotorProfile.calculate(getElevatorMotorProfileTime());
        ArmConstants.MASTER_ELEVATOR_MOTOR.getPIDController().setP(ArmConstants.ELEVATOR_PID_CONTROLLER.getP());
        ArmConstants.MASTER_ELEVATOR_MOTOR.getPIDController().setI(ArmConstants.ELEVATOR_PID_CONTROLLER.getI());
        ArmConstants.MASTER_ELEVATOR_MOTOR.getPIDController().setD(ArmConstants.ELEVATOR_PID_CONTROLLER.getD());
    }

    private double getAngleMotorProfileTime() {
        return Timer.getFPGATimestamp() - lastAngleMotorProfileGenerationTime;
    }

    private double getElevatorMotorProfileTime() {
        return Timer.getFPGATimestamp() - lastElevatorMotorProfileGenerationTime;
    }

    private Rotation2d getAngleMotorPositionDegrees() {
        return Rotation2d.fromRotations(angleEncoder.getPosition().getValue());
    }

    private double getElevatorMotorPositionRevolutions() {
        return magTicksToRevolutions(elevatorEncoder.getSelectedSensorPosition());
    }

    private double getAngleMotorVelocityInDegreesPerSeconds() {
        return angleEncoder.getVelocity().getValue();
    }

    private double getElevatorMotorVelocityInRevolutionsPerSecond() {
        return magTicksToRevolutions(elevatorEncoder.getSelectedSensorVelocity());
    }

    private void stopElevatorMotors() {
        masterElevatorMotor.stopMotor();
        followerElevatorMotor.stopMotor();
    }

    private void stopAngleMotors() {
        masterAngleMotor.stopMotor();
        followerAngleMotor.stopMotor();
    }
}

