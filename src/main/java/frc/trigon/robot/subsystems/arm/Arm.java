package frc.trigon.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.utilities.Conversions;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    private final static Arm INSTANCE = new Arm();
    private final ArmIO armIO = ArmIO.generateIO();
    private final ArmInputsAutoLogged armInputs = new ArmInputsAutoLogged();

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

    @Override
    public void periodic() {
        armIO.updateInputs(armInputs);
        Logger.processInputs("Arm", armInputs);
        updateMechanism();
    }

    void generateAngleMotorProfile(Rotation2d targetAngle, double speedPercentage) {
        angleMotorProfile = new TrapezoidProfile(
                Conversions.scaleConstraints(ArmConstants.ANGLE_CONSTRAINTS, speedPercentage),
                new TrapezoidProfile.State(targetAngle.getDegrees(), 0),
                new TrapezoidProfile.State(armInputs.anglePositionDegrees, armInputs.angleVelocityDegreesPerSecond)
        );
        lastAngleMotorProfileGenerationTime = Timer.getFPGATimestamp();
    }

    void generateElevatorMotorProfile(double targetElevatorPositionMeters, double speedPercentage) {
        elevatorMotorProfile = new TrapezoidProfile(
                Conversions.scaleConstraints(ArmConstants.ELEVATOR_CONSTRAINTS, speedPercentage),
                new TrapezoidProfile.State(targetElevatorPositionMeters, 0),
                new TrapezoidProfile.State(armInputs.elevatorPositionMeters, armInputs.elevatorVelocityMetersPerSecond)
        );
        lastElevatorMotorProfileGenerationTime = Timer.getFPGATimestamp();
    }

    void setTargetAngleFromProfile() {
        if (angleMotorProfile == null) {
            armIO.stopAngleMotors();
            return;
        }

        TrapezoidProfile.State targetState = angleMotorProfile.calculate(getAngleProfileTime());
        ArmConstants.TARGET_ARM_POSITION_LIGAMENT.setAngle(targetState.position);
        armIO.setTargetAngleState(targetState);
    }

    void setTargetElevatorPositionFromProfile() {
        if (elevatorMotorProfile == null) {
            armIO.stopElevatorMotors();
            return;
        }

        TrapezoidProfile.State targetState = elevatorMotorProfile.calculate(getElevatorProfileTime());
        ArmConstants.TARGET_ARM_POSITION_LIGAMENT.setLength(targetState.position + ArmConstants.RETRACTED_ARM_LENGTH_METERS);
        armIO.setTargetElevatorState(targetState);
    }

    boolean isElevatorRising(double targetElevatorPositionMeters) {
        return targetElevatorPositionMeters >= getElevatorPositionMeters();
    }

    boolean atAngle(Rotation2d targetAngle) {
        return Math.abs(armInputs.anglePositionDegrees - targetAngle.getDegrees()) <= ArmConstants.ANGLE_TOLERANCE_DEGREES;
    }

    boolean atElevatorPositionMeters(double targetElevatorPositionMeters) {
        return Math.abs(armInputs.elevatorPositionMeters - targetElevatorPositionMeters) <= ArmConstants.ELEVATOR_TOLERANCE_METERS;
    }

    private double getElevatorPositionMeters() {
        return armInputs.elevatorPositionMeters;
    }

    private double getAngleProfileTime() {
        return Timer.getFPGATimestamp() - lastAngleMotorProfileGenerationTime;
    }

    private double getElevatorProfileTime() {
        return Timer.getFPGATimestamp() - lastElevatorMotorProfileGenerationTime;
    }

    private void updateMechanism() {
        ArmConstants.ARM_LIGAMENT.setLength(armInputs.elevatorPositionMeters + ArmConstants.RETRACTED_ARM_LENGTH_METERS);
        ArmConstants.ARM_LIGAMENT.setAngle(armInputs.anglePositionDegrees);
        Logger.recordOutput("ArmMechanism", ArmConstants.ARM_MECHANISM);
    }
}