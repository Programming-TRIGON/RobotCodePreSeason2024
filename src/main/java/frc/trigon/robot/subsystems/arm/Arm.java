package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
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

    private TrapezoidProfile
            angleMotorProfile = null,
            elevatorMotorProfile = null;
    private double
            lastAngleProfileGenerationTime,
            lastElevatorProfileGenerationTime;

    public static Arm getInstance() {
        return INSTANCE;
    }

    private Arm() {
    }

    public Command getSetTargetArmAngleCommand(Rotation2d angle){
        return new FunctionalCommand(
                () -> generateAngleMotorProfile(angle),
                this::setTargetAngleFromProfile,
                (interrupted) -> {
                },
                () -> false,
                this
        );
    }

    public Command getSetTargetArmElevatorCommand(Rotation2d angle){
        return new FunctionalCommand(
                () -> generateElevatorMotorProfile(angle.getDegrees()),
                this::setTargetElevatorFromProfile,
                (interrupted) -> {
                },
                () -> false,
                this
        );
    }

    private void generateAngleMotorProfile(Rotation2d targetAngle) {
        angleMotorProfile = new TrapezoidProfile(
                ArmConstants.ANGLE_CONSTRAINS,
                new TrapezoidProfile.State(targetAngle.getDegrees(), 0),
                new TrapezoidProfile.State(getAnglePosition().getDegrees(), getAngleVelocityPerSecond())
        );
        lastAngleProfileGenerationTime = Timer.getFPGATimestamp();
    }

    private void generateElevatorMotorProfile(double targetPosition) {
        elevatorMotorProfile = new TrapezoidProfile(
                ArmConstants.ELEVATOR_CONSTRAINS,
                new TrapezoidProfile.State(targetPosition, 0),
                new TrapezoidProfile.State(getElevatorPositionRevolutions(), getElevatorVelocityRevolutionsPerSecond())
        );
        lastElevatorProfileGenerationTime = Timer.getFPGATimestamp();
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

    private double getAngleMotorProfileTime() {
        return Timer.getFPGATimestamp() - lastAngleProfileGenerationTime;
    }

    private double getElevatorMotorProfileTime() {
        return Timer.getFPGATimestamp() - lastElevatorProfileGenerationTime;
    }

    private Rotation2d getAnglePosition() {
        double position = ArmConstants.ANGLE_MOTOR_POSITION_SIGNAL.refresh().getValue();
        return Rotation2d.fromRotations(position);
    }

    private double getElevatorPositionRevolutions() {
        return Conversions.magTicksToRevolutions(elevatorEncoder.getSelectedSensorPosition());
    }

    private double calculateAngleMotorOutput(TrapezoidProfile.State targetState){
        double pidOutput = ArmConstants.ANGLE_PID_CONTROLLER.calculate(
                getAnglePosition().getDegrees(),
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
                getElevatorPositionRevolutions(),
                targetState.position
        );
        double feedforward = ArmConstants.ELEVATOR_MOTOR_FEEDFORWARD.calculate(targetState.velocity);
        return feedforward + pidOutput;
    }

    private double getAngleVelocityPerSecond() {
        double position = ArmConstants.ANGLE_MOTOR_VELOCITY_SIGNAL.refresh().getValue();
        return Conversions.revolutionsToDegrees(position);
    }

    private double getElevatorVelocityRevolutionsPerSecond() {
        double velocity = Conversions.perHundredMsToPerSecond(elevatorEncoder.getSelectedSensorVelocity());
        return Conversions.degreesToRevolutions(velocity);
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
}
