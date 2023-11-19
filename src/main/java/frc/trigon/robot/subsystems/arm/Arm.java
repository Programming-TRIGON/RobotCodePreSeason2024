package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.utilities.Conversions;

public class Arm extends SubsystemBase {
    private final static Arm INSTANCE = new Arm();

    private final CANSparkMax
            masterAngleMotor = ArmConstants.MASTER_ANGLE_MOTOR,
            followerAngleMotor = ArmConstants.FOLLOWER_ANGLE_MOTOR,
            masterElevatorMotor = ArmConstants.MASTER_ELEVATOR_MOTOR,
            followerElevatorMotor = ArmConstants.FOLLOWER_ELEVATOR_MOTOR;
    private final TalonSRX elevatorEncoder = ArmConstants.ELEVATOR_ENCODER;
    private final CANcoder angleEncoder = ArmConstants.ANGLE_ENCODER;

    private TrapezoidProfile
            angleMotorProfile = null,
            elevatorMotorProfile = null;
    private double
            lastAngleProfileGeneration,
            lastElevatorProfileGeneration;

    public static Arm getInstance() {
        return INSTANCE;
    }

    private Arm() {
    }

    private void stopAngleMotors(){
        masterAngleMotor.stopMotor();
        followerAngleMotor.stopMotor();
    }

    private void stopElevatorMotors(){
        masterElevatorMotor.stopMotor();
        followerElevatorMotor.stopMotor();
    }

    private void setAngleMotorsVoltage(double voltage) {
        masterAngleMotor.setVoltage(voltage);
        followerAngleMotor.setVoltage(voltage);
    }

    private void setElevatorMotorsVoltage(double voltage) {
        masterElevatorMotor.setVoltage(voltage);
        followerElevatorMotor.setVoltage(voltage);
    }

    private double getAngleMotorProfileTime() {
        return Timer.getFPGATimestamp() - lastAngleProfileGeneration;
    }

    private double getElevatorMotorProfileTime() {
        return Timer.getFPGATimestamp() - lastElevatorProfileGeneration;
    }

    private double calculateAngleMotorOutput(TrapezoidProfile.State targetState){
        double pidOutput = ArmConstants.ANGLE_PID_CONTROLLER.calculate(
                getAngleMotorPositionDegrees().getDegrees(),
                targetState.position
        );
        double feedforward = ArmConstants.ANGLE_MOTOR_FEEDFORWARD.calculate(
                Units.degreesToRadians(targetState.position),
                Units.degreesToRadians(targetState.velocity)
        );
        return feedforward + pidOutput;
    }
    private double calculateElevatorMotorOutput(TrapezoidProfile.State targetState){
        double pidOutput = ArmConstants.ELEVATOR_PID_CONTROLLER.calculate(
                getElevatorMotorPositionConversions().getDegrees(),
                targetState.position
        );
        double feedforward = ArmConstants.ELEVATOR_MOTOR_FEEDFORWARD.calculate(targetState.velocity);
        return feedforward + pidOutput;
    }

    private void generateAngleMotorProfile(Rotation2d targetAngle) {
        angleMotorProfile = new TrapezoidProfile(
                ArmConstants.ANGLE_CONSTRAINS,
                new TrapezoidProfile.State(targetAngle.getDegrees(), 0),
                new TrapezoidProfile.State(getAngleMotorPositionDegrees().getDegrees(), getAngleMotorVelocityPerSecond())
        );
        lastAngleProfileGeneration = Timer.getFPGATimestamp();
    }

    private void generateElevatorMotorProfile(double position) {
        elevatorMotorProfile = new TrapezoidProfile(
                ArmConstants.ELEVATOR_CONSTRAINS,
                new TrapezoidProfile.State(position, 0),
                new TrapezoidProfile.State(getElevatorMotorPositionConversions().getDegrees(), getAngleMotorVelocityPerSecond())
        );
        lastElevatorProfileGeneration = Timer.getFPGATimestamp();
    }

    private void setTargetAngleFromProfile() {
        if (angleMotorProfile == null) {
            stopAngleMotors();
            return;
        }
        TrapezoidProfile.State targetState = angleMotorProfile.calculate(getAngleMotorProfileTime());
        setAngleMotorsVoltage(calculateAngleMotorOutput(targetState));
    }

    private void setTargetElevatorFromProfile() {
        if (elevatorMotorProfile == null) {
            stopElevatorMotors();
            return;
        }
        TrapezoidProfile.State targetState = elevatorMotorProfile.calculate(getElevatorMotorProfileTime());
        setElevatorMotorsVoltage(calculateElevatorMotorOutput(targetState));
    }
    
    private Rotation2d getAngleMotorPositionDegrees() {
        double position = ArmConstants.ANGLE_MOTOR_POSITION_SIGNAL.refresh().getValue();
        return new Rotation2d(position);
    }

    private Rotation2d getElevatorMotorPositionConversions() {
        return new Rotation2d(elevatorEncoder.getSelectedSensorPosition());
    }

    private double getAngleMotorVelocityPerSecond() {
        double velocity = ArmConstants.ANGLE_MOTOR_POSITION_SIGNAL.refresh().getValue();
        return Conversions.revolutionsToDegrees(velocity);
    }

    private double getElevatorMotorVelocityUnit() {
        double velocity = elevatorEncoder.getSelectedSensorVelocity();
        return Conversions.degreesToRevolutions(velocity);
    }
}
