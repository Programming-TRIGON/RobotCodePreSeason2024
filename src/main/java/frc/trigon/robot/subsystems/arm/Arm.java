package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private final static Arm INSTANCE = new Arm();

    private final CANSparkMax
            masterAngleMotor = ArmConstants.MASTER_ANGLE_MOTOR,
            followerAngleMotor = ArmConstants.FOLLOWER_ANGLE_MOTOR,
            masterElevatorMotor = ArmConstants.MASTER_ELEVATOR_MOTOR,
            followerElevatorMotor = ArmConstants.FOLLOWER_ELEVATOR_MOTOR;
    private final TalonSRX elevatorEncoder = ArmConstants.ELEVATOR_ENCODER;
    private final CANcoder angleEncoder = ArmConstants.ANGLE_ENCODER;

    private final ArmFeedforward angleMotorFeedforward = ArmConstants.ANGLE_MOTOR_FEEDFORWARD;
    private final ElevatorFeedforward elevatorMotorFeedforward = ArmConstants.ELEVATOR_MOTOR_FEEDFORWARD;

    private TrapezoidProfile
            angleMotorProfile = null,
            elevatorMotorProfile = null;
    private double
            lastAngleProfileGeneration,
            lastElevatorProfileGeneration;

    public static Arm getInstance() {
        return INSTANCE;
    }

    private double getAngleMotorPositionDegrees() {
        return angleEncoder.getPosition().getValue();
    }

    private double getElevatorMotorPositionDegrees() {
        return elevatorEncoder.getSelectedSensorPosition();
    }

    private double getAngleMotorVelocity() {
        return angleEncoder.getVelocity().getValue();
    }

    private double getElevatorMotorVelocity() {
        return elevatorEncoder.getSelectedSensorVelocity();
    }

    private double getAngleMotorProfileTime() {
        return Timer.getFPGATimestamp() - lastAngleProfileGeneration;
    }

    private double getElevatorMotorProfileTime() {
        return Timer.getFPGATimestamp() - lastElevatorProfileGeneration;
    }

    private void generateAngleMotorProfile(Rotation2d targetAngle) {
        angleMotorProfile = new TrapezoidProfile(
                ArmConstants.ANGLE_CONSTRAINS,
                new TrapezoidProfile.State(targetAngle.getDegrees(), 0),
                new TrapezoidProfile.State(getAngleMotorPositionDegrees(), getElevatorMotorVelocity())
        );
        lastElevatorProfileGeneration = Timer.getFPGATimestamp();
    }

    private void generateElevatorMotorProfile(Rotation2d targetAngle) {
        elevatorMotorProfile = new TrapezoidProfile(
                ArmConstants.ELEVATOR_CONSTRAINS,
                new TrapezoidProfile.State(targetAngle.getDegrees(), 0),
                new TrapezoidProfile.State(getElevatorMotorPositionDegrees(), getAngleMotorVelocity())
        );
        lastAngleProfileGeneration = Timer.getFPGATimestamp();
    }

    private void setTargetAngleFromProfile() {
        if (angleMotorProfile == null) {
            masterAngleMotor.stopMotor();
            followerAngleMotor.stopMotor();
            return;
        }
        TrapezoidProfile.State targetState = angleMotorProfile.calculate(getAngleMotorProfileTime());
        double pidOutput = ArmConstants.ANGLE_PID_CONTROLLER.calculate(
                getAngleMotorPositionDegrees(),
                targetState.position
        );
        double feedForward = ArmConstants.ANGLE_MOTOR_FEEDFORWARD.calculate(
                Units.degreesToRadians(targetState.position),
                Units.degreesToRadians(targetState.velocity)
        );
        masterAngleMotor.setVoltage(pidOutput + feedForward);
        followerAngleMotor.setVoltage(pidOutput + feedForward);
    }

    private void setTargetElevatorFromProfile() {
        if (elevatorMotorProfile == null) {
            masterElevatorMotor.stopMotor();
            followerElevatorMotor.stopMotor();
            return;
        }
        TrapezoidProfile.State targetState = elevatorMotorProfile.calculate(getElevatorMotorProfileTime());
        double pidOutput = ArmConstants.ELEVATOR_PID_CONTROLLER.calculate(
                getElevatorMotorPositionDegrees(),
                targetState.position
        );
        double feedForward = ArmConstants.ELEVATOR_MOTOR_FEEDFORWARD.calculate(
                Units.degreesToRadians(targetState.position),
                Units.degreesToRadians(targetState.velocity)
        );
        masterElevatorMotor.setVoltage(pidOutput + feedForward);
        followerElevatorMotor.setVoltage(pidOutput + feedForward);
    }

    private Arm() {
    }
}
