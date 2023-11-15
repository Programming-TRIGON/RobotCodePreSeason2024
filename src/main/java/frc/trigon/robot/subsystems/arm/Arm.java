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
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.utilities.Conversions;

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

    public Command setArmTarget(ArmConstants.ArmState targetState){
        return new StartEndCommand(
                () -> getSetTargetAngleCommand(targetState.angle),
                () -> getSetTargetElevatorPositionCommand(targetState.elevatorPosition)
        );
    }

    public Command getSetTargetAngleCommand(Rotation2d targetAngle) {
        return new FunctionalCommand(
                () -> generateAngleMotorProfile(targetAngle),
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
                this::setTargetElevatorPositionFromProfile,
                (interrupted) -> {
                },
                () -> false,
                this
        );
    }

    private void setTargetAngleFromProfile() {
        if (angleMotorProfile == null) {
            stopAngleMotors();
            return;
        }

        TrapezoidProfile.State targetState = angleMotorProfile.calculate(getAngleMotorProfileTime());
        double voltage = calculateAngleMotorOutput(targetState);
        setAngleMotorsVoltage(voltage);
    }

    private void setTargetElevatorPositionFromProfile() {
        if (elevatorMotorProfile == null) {
            stopElevatorMotors();
            return;
        }

        TrapezoidProfile.State targetState = elevatorMotorProfile.calculate(getElevatorMotorProfileTime());
        double voltage = calculateElevatorMotorOutput(targetState);
        setElevatorMotorsVoltage(voltage);
    }

    private void setAngleMotorsVoltage(double voltage){
        masterAngleMotor.setVoltage(voltage);
        followerAngleMotor.setVoltage(voltage);
    }

    private void setElevatorMotorsVoltage(double voltage){
        masterElevatorMotor.setVoltage(voltage);
        followerElevatorMotor.setVoltage(voltage);
    }

    private double calculateAngleMotorOutput(TrapezoidProfile.State targetState){
        double pidOutput = ArmConstants.ANGLE_PID_CONTROLLER.calculate(
                getAngleMotorPosition().getDegrees(),
                targetState.position
        );
        double feedForward = ArmConstants.ANGLE_FEEDFORWARD.calculate(
                Units.degreesToRadians(targetState.position),
                Units.degreesToRadians(targetState.velocity)
        );

        return pidOutput + feedForward;
    }

    private double calculateElevatorMotorOutput(TrapezoidProfile.State targetState){
        double pidOutput = ArmConstants.ELEVATOR_PID_CONTROLLER.calculate(
                getElevatorMotorPositionRevolutions(),
                targetState.position
        );
        double feedForward = ArmConstants.ELEVATOR_FEEDFORWARD.calculate(
                Units.rotationsToDegrees(targetState.velocity)
        );

        return pidOutput + feedForward;
    }

    private void generateAngleMotorProfile(Rotation2d targetAngle) {
        angleMotorProfile = new TrapezoidProfile(
                ArmConstants.ANGLE_CONSTRAINTS,
                new TrapezoidProfile.State(targetAngle.getDegrees(), 0),
                new TrapezoidProfile.State(getAngleMotorPosition().getDegrees(), getAngleMotorVelocityDegreesPerSecond())
        );

        lastAngleMotorProfileGenerationTime = Timer.getFPGATimestamp();
    }

    private void generateElevatorMotorProfile(double targetElevatorPosition) {
        elevatorMotorProfile = new TrapezoidProfile(
                ArmConstants.ELEVATOR_CONSTRAINTS,
                new TrapezoidProfile.State(targetElevatorPosition, 0),
                new TrapezoidProfile.State(getElevatorMotorPositionRevolutions(), getElevatorVelocityRevolutionsPerSecond())
        );

        lastElevatorMotorProfileGenerationTime = Timer.getFPGATimestamp();
    }

    private double getAngleMotorProfileTime() {
        return Timer.getFPGATimestamp() - lastAngleMotorProfileGenerationTime;
    }

    private double getElevatorMotorProfileTime() {
        return Timer.getFPGATimestamp() - lastElevatorMotorProfileGenerationTime;
    }

    private Rotation2d getAngleMotorPosition() {
        double positionRevolutions = ArmConstants.ANGLE_MOTOR_POSITION_SIGNAL.refresh().getValue();
        return Rotation2d.fromRotations(positionRevolutions);
    }

    private double getElevatorMotorPositionRevolutions() {
        return Conversions.magTicksToRevolutions(elevatorEncoder.getSelectedSensorPosition());
    }

    private double getAngleMotorVelocityDegreesPerSecond() {
        double positionRevolutions = ArmConstants.ANGLE_MOTOR_VELOCITY_SIGNAL.refresh().getValue();
        return Conversions.revolutionsToDegrees(positionRevolutions);
    }

    private double getElevatorVelocityRevolutionsPerSecond() {
        double magTicksPerSecond = Conversions.perHundredMsToPerSecond(elevatorEncoder.getSelectedSensorVelocity());
        return Conversions.magTicksToRevolutions(magTicksPerSecond);
    }

    private void stopAngleMotors() {
        masterAngleMotor.stopMotor();
        followerAngleMotor.stopMotor();
    }

    private void stopElevatorMotors() {
        masterElevatorMotor.stopMotor();
        followerElevatorMotor.stopMotor();
    }
}

