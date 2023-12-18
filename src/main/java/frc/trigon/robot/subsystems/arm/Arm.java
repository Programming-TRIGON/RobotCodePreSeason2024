package frc.trigon.robot.subsystems.arm;


import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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
            masterElevatorMotor = ArmConstants.MASTER_ELEVATOR_MOTOR,
            followerElevatorMotor = ArmConstants.MASTER_ELEVATOR_MOTOR,
            masterAngleMotor = ArmConstants.MASTER_ELEVATOR_MOTOR,
            followerAngleMotor = ArmConstants.MASTER_ELEVATOR_MOTOR;
    private final TalonSRX elevatorEncoder = ArmConstants.ELEVATOR_ENCODER;
    private TrapezoidProfile angleMotorProfile = null;
    private TrapezoidProfile elevatorMotorProfile = null;
    private double lastAngleMotorProfileGeneration;
    private double lastElevatorMotorProfileGeneration;

    public static Arm getInstance() {
        return INSTANCE;
    }

    private Arm() {
    }

    void generateElevatorMotorProfile(double targetElevatorMeters) {
        elevatorMotorProfile = new TrapezoidProfile(
                ArmConstants.ANGLE_CONSTRAINTS,
                new TrapezoidProfile.State(targetElevatorMeters, 0),
                new TrapezoidProfile.State(getElevatorPositionMeters(), getElevatorVelocityMetersPerSeconds())
        );

        lastElevatorMotorProfileGeneration = Timer.getFPGATimestamp();
    }

    void generateAngleMotorProfile(Rotation2d targetAngle) {
        angleMotorProfile = new TrapezoidProfile(
                ArmConstants.ANGLE_CONSTRAINTS,
                new TrapezoidProfile.State(targetAngle.getDegrees(), 0),
                new TrapezoidProfile.State(getAnglePosition().getDegrees(), getAngleVelocityDegreesPerSeconds())
        );

        lastAngleMotorProfileGeneration = Timer.getFPGATimestamp();
    }

    private Rotation2d getAnglePosition() {
        double positionRevolutions = ArmConstants.ANGLE_ENCODER_POSITION_SIGNAL.refresh().getValue();
        return Rotation2d.fromRotations(positionRevolutions);
    }

    private double getElevatorPositionMeters() {
        double elevatorPositionMagTicks = elevatorEncoder.getSelectedSensorPosition();
        double elevatorPositionRevolution = Conversions.magTicksToRevolutions(elevatorPositionMagTicks);
        return elevatorPositionRevolution / ArmConstants.ELEVATOR_METERS_PER_REVOLUTION;
    }

    private double getAngleVelocityDegreesPerSeconds() {
        return Conversions.revolutionsToDegrees(ArmConstants.ANGLE_ENCODER_VELOCITY_SIGNAL.refresh().getValue());
    }

    private double getElevatorVelocityMetersPerSeconds(){
        double elevatorVelocityMagTicksPer100ms = elevatorEncoder.getSelectedSensorVelocity();
        double elevatorVelocityMagTicksPerSecond = Conversions.perHundredMsToPerSecond(elevatorVelocityMagTicksPer100ms);
        double elevatorVelocityRevolutionPerSecond = Conversions.magTicksToRevolutions(elevatorVelocityMagTicksPerSecond);
        return elevatorVelocityRevolutionPerSecond / ArmConstants.ELEVATOR_METERS_PER_REVOLUTION;
    }

    void setTargetAngleFromProfile() {
        if (angleMotorProfile == null) {
            masterAngleMotor.stopMotor();
            followerAngleMotor.stopMotor();
            return;
        }

        TrapezoidProfile.State targetState = angleMotorProfile.calculate(getAngleProfileTime());
        setAngleMotorsVoltage(calculateAngleOutput(targetState));
    }

    void setTargetElevatorPositionFromProfile() {
        if (angleMotorProfile == null) {
            masterElevatorMotor.stopMotor();
            followerElevatorMotor.stopMotor();
            return;
        }

        TrapezoidProfile.State targetState = elevatorMotorProfile.calculate(getElevatorProfileTime());
        setElevatorMotorsVoltage(calculateElevatorOutput(targetState));
    }

    private double calculateAngleOutput(TrapezoidProfile.State targetState){
        double pidOutput = ArmConstants.ANGLE_PID_CONTROLLER.calculate(
                getAnglePosition().getDegrees(),
                targetState.position
        );

        double feedforward = ArmConstants.ANGLE_MOTOR_FEEDFORWARD.calculate(
                Units.degreesToRadians(targetState.position),
                Units.degreesToRadians(targetState.velocity)
        );

        return pidOutput + feedforward;
    }

    private double calculateElevatorOutput(TrapezoidProfile.State targetState){
        double pidOutput = ArmConstants.ELEVATOR_PID_CONTROLLER.calculate(
                getAnglePosition().getDegrees(),
                targetState.position
        );

        double feedforward = ArmConstants.ELEVATOR_MOTOR_FEEDFORWARD.calculate(
                targetState.velocity
        );

        return pidOutput + feedforward;
    }

    private void setAngleMotorsVoltage(double outputVoltage){
        masterAngleMotor.setVoltage(outputVoltage);
        followerAngleMotor.setVoltage(outputVoltage);
    }

    private void setElevatorMotorsVoltage(double outputVoltage){
        masterElevatorMotor.setVoltage(outputVoltage);
        followerElevatorMotor.setVoltage(outputVoltage);
    }

    boolean atAngle(Rotation2d angle){
        return  Math.abs(angle.getDegrees() - getAnglePosition().getDegrees()) < ArmConstants.ANGLE_TOLERANCE;
    }

    boolean atElevatorTargetMeters(double targetElevatorMeters){
        return Math.abs(targetElevatorMeters - getElevatorPositionMeters()) < ArmConstants.ELEVATOR_TOLERANCE;
    }

    boolean isElevatorOpening(double targetElevatorMeters){
        return targetElevatorMeters > getElevatorPositionMeters();
    }

    private double getAngleProfileTime() {
        return Timer.getFPGATimestamp() - lastAngleMotorProfileGeneration;
    }

    private double getElevatorProfileTime(){
        return Timer.getFPGATimestamp() - lastElevatorMotorProfileGeneration;
    }
}