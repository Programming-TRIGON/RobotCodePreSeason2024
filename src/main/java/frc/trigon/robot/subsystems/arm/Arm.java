package frc.trigon.robot.subsystems.arm;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.utilities.Conversions;

public class Arm extends SubsystemBase {
    private final CANSparkMax
            MASTER_ELEVATOR_MOTOR = ArmConstants.MASTER_ELEVATOR_MOTOR,
            FOLLOWER_ELEVATOR_MOTOR = ArmConstants.MASTER_ELEVATOR_MOTOR,
            MASTER_ANGLE_MOTOR = ArmConstants.MASTER_ELEVATOR_MOTOR,
            FOLLOWER_ANGLE_MOTOR = ArmConstants.FOLLOWER_ELEVATOR_MOTOR;
    private final TalonSRX ELEVATOR_TALON_SRX_ENCODER = ArmConstants.ELEVATOR_TALON_SRX_ENCODER;
    private final static Arm INSTANCE = new Arm();

    private TrapezoidProfile
            elevatorMotorProfile = null,
            angleMotorProfile = null;
    private double lastAngleMotorProfileGenerationTime, lastElevatorMotorProfileGenerationTime;

    public static Arm getInstance() {
        return INSTANCE;
    }

    private Arm() {
    }

    void generateAngleMotorProfile(Rotation2d targetAngle) {
        angleMotorProfile = new TrapezoidProfile(
                ArmConstants.ANGLE_CONSTRAINTS,
                new TrapezoidProfile.State(targetAngle.getDegrees(), 0),
                new TrapezoidProfile.State(getAngleEncoderPosition().getDegrees(), getAngleVelocityDegreesPerSecond())
        );
        lastAngleMotorProfileGenerationTime = Timer.getFPGATimestamp();
    }

    void setTargetAngleFromProfile() {
        if (angleMotorProfile == null) {
            stopAngleMotors();
            return;
        }
        TrapezoidProfile.State targetState = angleMotorProfile.calculate(getAngleMotorProfileTime());
        setTargetAngleVoltage();
    }

    private void setTargetAngleVoltage(){
        MASTER_ANGLE_MOTOR.setVoltage(calculateAngleMotorOutput());
        FOLLOWER_ANGLE_MOTOR.setVoltage(calculateAngleMotorOutput());
    }

    private double calculateAngleMotorOutput() {
        TrapezoidProfile.State targetState = angleMotorProfile.calculate(getAngleMotorProfileTime());
        double pidOutput = ArmConstants.ANGLE_PID_CONTROLLER.calculate(
                getAngleEncoderPosition().getDegrees(),
                targetState.position
        );
        double feedforward = ArmConstants.ANGLE_MOTOR_FEEDFORWARD.calculate(
                Units.degreesToRadians(targetState.position),
                Units.degreesToRadians(targetState.velocity)
        );
        return pidOutput + feedforward;
    }

    void generateElevatorMotorProfile(Rotation2d targetElevatorPosition) {
        elevatorMotorProfile = new TrapezoidProfile(
                ArmConstants.ELEVATOR_CONSTRAINTS,
                new TrapezoidProfile.State(targetElevatorPosition.getDegrees(), 0),
                new TrapezoidProfile.State(getElevatorPositionSecondPerMeter(), getElevatorVelocityRevolutionPerSecond())
        );
        lastElevatorMotorProfileGenerationTime = Timer.getFPGATimestamp();
    }

    void setTargetElevatorFromProfile() {
        if (elevatorMotorProfile == null) {
            stopElevatorMotors();
            return;
        }
        setTargetElevatorVoltage();
    }

    private void setTargetElevatorVoltage(){
        MASTER_ELEVATOR_MOTOR.setVoltage(calculateElevatorMotorOutput());
        FOLLOWER_ELEVATOR_MOTOR.setVoltage(calculateElevatorMotorOutput());
    }

    private double calculateElevatorMotorOutput() {
        TrapezoidProfile.State targetState = elevatorMotorProfile.calculate(getElevatorMotorProfileTime());
        double pidOutput = ArmConstants.ELEVATOR_PID_CONTROLLER.calculate(
                getElevatorPositionSecondPerMeter(),
                targetState.position
        );
        double feedforward = ArmConstants.ELEVATOR_MOTOR_FEEDFORWARD.calculate(
                Units.degreesToRadians(targetState.position),
                Units.degreesToRadians(targetState.velocity)
        );
        return pidOutput + feedforward;
    }

    private double getElevatorMotorProfileTime() {
        return timer.getFPGATimestamp() - lastElevatorMotorProfileGenerationTime;
    }

    private double getElevatorPositionSecondPerMeter() {
        return Conversions.perHundredMsToPerSecond(Conversions.magTicksToRevolutions(ArmConstants.METER_PER_REVOLUTION * ELEVATOR_TALON_SRX_ENCODER.getSelectedSensorPosition()));
    }

    private double getElevatorVelocityRevolutionPerSecond() {
        return Conversions.perHundredMsToPerSecond(Conversions.magTicksToRevolutions(ELEVATOR_TALON_SRX_ENCODER.getSelectedSensorVelocity()));
    }

    private double getAngleMotorProfileTime() {
        return Timer.getFPGATimestamp() - lastAngleMotorProfileGenerationTime;
    }

    private Rotation2d getAngleEncoderPosition() {
        return Rotation2d.fromRotations(ArmConstants.ANGEL_ENCODER_POSITION_SIGNAL.refresh().getValue());
    }

    private double getAngleVelocityDegreesPerSecond() {
        return Conversions.revolutionsToDegrees(ArmConstants.ANGLE_ENCODER_VELOCITY_SIGNAL.refresh().getValue());
    }

    private void stopAngleMotors() {
        MASTER_ANGLE_MOTOR.stopMotor();
        FOLLOWER_ANGLE_MOTOR.stopMotor();
    }

    private void stopElevatorMotors() {
        MASTER_ELEVATOR_MOTOR.stopMotor();
        FOLLOWER_ELEVATOR_MOTOR.stopMotor();
    }
}

