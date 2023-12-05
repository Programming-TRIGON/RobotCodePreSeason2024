package frc.trigon.robot.subsystems.arm;


import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.subsystems.sideshooter.SideShooterConstants;
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
    private double lastAngleMotorProfileGeneration;

    private TrapezoidProfile elevatorMotorProfile = null;
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
                new TrapezoidProfile.State(getElevatorPositionMeters(), getElevatorVelocityInMetersPerSeconds())
        );
    }

    void generateAngleMotorProfile(Rotation2d targetAngle) {
        angleMotorProfile = new TrapezoidProfile(
                ArmConstants.ANGLE_CONSTRAINTS,
                new TrapezoidProfile.State(targetAngle.getDegrees(), 0),
                new TrapezoidProfile.State(getAnglePosition().getDegrees(), getAngleVelocityInDegreesPerSeconds())
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

    private double getAngleVelocityInDegreesPerSeconds() {
        return Conversions.revolutionsToDegrees(ArmConstants.ANGLE_ENCODER_VELOCITY_SIGNAL.refresh().getValue());
    }

    private double getElevatorVelocityInMetersPerSeconds(){
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
        double output = ArmConstants.ANGLE_PID_CONTROLLER.calculate(getAnglePosition().getDegrees(), targetState.position);
        masterAngleMotor.setVoltage(output);
        followerAngleMotor.setVoltage(output);
    }

    void setTargetElevatorPositionFromProfile() {
        if (angleMotorProfile == null) {
            masterElevatorMotor.stopMotor();
            followerElevatorMotor.stopMotor();
            return;
        }

        TrapezoidProfile.State targetState = elevatorMotorProfile.calculate(getElevatorProfileTime());
        double output = ArmConstants.ELEVATOR_PID_CONTROLLER.calculate(getElevatorPositionMeters(), targetState.position);
        masterElevatorMotor.setVoltage(output);
        followerElevatorMotor.setVoltage(output);
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