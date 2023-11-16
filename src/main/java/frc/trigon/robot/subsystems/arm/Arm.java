package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.utilities.Conversions;

import java.util.Set;

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

    public Command getSetTargetArmStateCommand(ArmConstants.ArmState targetState) {
        return new DeferredCommand(
                () -> getCurrentSetTargetArmStateCommand(targetState),
                Set.of(this)
        );
    }

    private Command getCurrentSetTargetArmStateCommand(ArmConstants.ArmState targetState) {
        if (targetState.elevatorPosition >= getElevatorPositionRevolutions()) {
            return new SequentialCommandGroup(
                    getSetTargetAngleCommand(targetState.angle),
                    getSetTargetElevatorPositionCommand(targetState.elevatorPosition)
            );
        }
        return new SequentialCommandGroup(
                getSetTargetElevatorPositionCommand(targetState.elevatorPosition),
                getSetTargetAngleCommand(targetState.angle)
        );
    }

    private Command getSetTargetAngleCommand(Rotation2d angle) {
        return new FunctionalCommand(
                () -> generateAngleMotorProfile(angle),
                this::setTargetAngleFromProfile,
                (interrupted) -> {
                },
                () -> false,
                this
        );
    }

    private Command getSetTargetElevatorPositionCommand(double targetElevatorPosition) {
        return new FunctionalCommand(
                () -> generateElevatorMotorProfile(targetElevatorPosition),
                this::setTargetElevatorPositionFromProfile,
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
                new TrapezoidProfile.State(getAnglePositionDegrees().getDegrees(), getAngleVelocityDegreesPerSecond())
        );
        lastAngleMotorProfileGenerationTime = Timer.getFPGATimestamp();
    }

    private void generateElevatorMotorProfile(double targetElevatorPosition) {
        elevatorMotorProfile = new TrapezoidProfile(
                ArmConstants.ELEVATOR_CONSTRAINTS,
                new TrapezoidProfile.State(targetElevatorPosition, 0),
                new TrapezoidProfile.State(getElevatorPositionRevolutions(), getElevatorVelocityRevolutionsPerSecond())
        );
        lastElevatorMotorProfileGenerationTime = Timer.getFPGATimestamp();
    }

    private void setTargetAngleFromProfile() {
        if (angleMotorProfile == null) {
            stopAngleMotors();
            return;
        }

        TrapezoidProfile.State targetState = angleMotorProfile.calculate(getAngleProfileTime());
        setAngleMotorsVoltage(calculateAngleMotorVoltage(targetState));
    }

    private void setTargetElevatorPositionFromProfile() {
        if (elevatorMotorProfile == null) {
            stopElevatorMotors();
            return;
        }

        TrapezoidProfile.State targetState = elevatorMotorProfile.calculate(getElevatorProfileTime());
        setElevatorMotorsVoltage(calculateElevatorMotorVoltage(targetState));
    }

    private double getAngleProfileTime() {
        return Timer.getFPGATimestamp() - lastAngleMotorProfileGenerationTime;
    }

    private double getElevatorProfileTime() {
        return Timer.getFPGATimestamp() - lastElevatorMotorProfileGenerationTime;
    }

    private double getElevatorPositionRevolutions() {
        return Conversions.magTicksToRevolutions(elevatorEncoder.getSelectedSensorPosition());
    }

    private Rotation2d getAnglePositionDegrees() {
        return Rotation2d.fromDegrees(Conversions.revolutionsToDegrees(ArmConstants.ANGLE_ENCODER_VELOCITY_SIGNAL.refresh().getValue()));
    }

    private double getAngleVelocityDegreesPerSecond() {
        return Conversions.revolutionsToDegrees(ArmConstants.ANGLE_ENCODER_VELOCITY_SIGNAL.refresh().getValue());
    }

    private double getElevatorVelocityRevolutionsPerSecond() {
        double velocityRevolutions = Conversions.magTicksToRevolutions(elevatorEncoder.getSelectedSensorVelocity());
        return Conversions.perHundredMsToPerSecond(velocityRevolutions);
    }

    private void stopElevatorMotors() {
        masterElevatorMotor.stopMotor();
        followerElevatorMotor.stopMotor();
    }

    private void stopAngleMotors() {
        masterAngleMotor.stopMotor();
        followerAngleMotor.stopMotor();
    }

    private double calculateAngleMotorVoltage(TrapezoidProfile.State targetState) {
        double pidOutput = ArmConstants.ANGLE_PID_CONTROLLER.calculate(
                ArmConstants.ANGLE_ENCODER_POSITION_SIGNAL.refresh().getValue(),
                targetState.position
        );
        double feedforward = ArmConstants.ANGLE_FEEDFORWARD.calculate(
                Units.degreesToRadians(targetState.position),
                Units.degreesToRadians(targetState.velocity)
        );
        return pidOutput + feedforward;
    }

    private double calculateElevatorMotorVoltage(TrapezoidProfile.State targetState) {
        double pidOutput = ArmConstants.ELEVATOR_PID_CONTROLLER.calculate(
                getElevatorPositionRevolutions(),
                targetState.position
        );
        double feedforward = ArmConstants.ELEVATOR_FEEDFORWARD.calculate(targetState.velocity);
        return pidOutput + feedforward;
    }

    private void setAngleMotorsVoltage(double voltage) {
        masterAngleMotor.setVoltage(voltage);
        followerAngleMotor.setVoltage(voltage);
    }

    private void setElevatorMotorsVoltage(double voltage) {
        masterElevatorMotor.setVoltage(voltage);
        followerElevatorMotor.setVoltage(voltage);
    }
}

