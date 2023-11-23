package frc.trigon.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.utilities.Conversions;
import org.littletonrobotics.junction.Logger;

import java.util.Set;

public class Arm extends SubsystemBase {
    private final static Arm INSTANCE = new Arm();
    private final ArmIO armIO = ArmIO.generateIO();
    private final ArmInputsAutoLogged armInputs = new ArmInputsAutoLogged();

    @Override
    public void periodic() {
        armIO.updateInputs(armInputs);
        Logger.processInputs("Arm", armInputs);
    }

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
        return getSetTargetArmStateCommand(targetState, 100, 100);
    }

    public Command getSetTargetArmPositionCommand(Rotation2d targetAngle, double targetElevatorPosition) {
        return getSetTargetArmPositionCommand(targetAngle, targetElevatorPosition, 100, 100);
    }

    /**
     * Creates a command that sets the target state of the arm.
     *
     * @param targetState             the target state of the arm
     * @param angleSpeedPercentage    the target speed for the angle motor as a percentage of the normal speed
     * @param elevatorSpeedPercentage the target speed for the elevator motor as a percentage of the normal speed
     * @return the command
     */
    public Command getSetTargetArmStateCommand(ArmConstants.ArmState targetState, double angleSpeedPercentage, double elevatorSpeedPercentage) {
        return getSetTargetArmPositionCommand(targetState.angle, targetState.elevatorPosition, angleSpeedPercentage, elevatorSpeedPercentage);
    }

    /**
     * Creates a command that sets the target position of the arm.
     *
     * @param targetAngle             the target angle of the arm
     * @param targetElevatorPosition  the target elevator position
     * @param angleSpeedPercentage    the target speed for angle motor as a percentage of normal speed
     * @param elevatorSpeedPercentage the target speed for elevator motor as a percentage of normal speed
     * @return the command
     */
    public Command getSetTargetArmPositionCommand(Rotation2d targetAngle, double targetElevatorPosition, double angleSpeedPercentage, double elevatorSpeedPercentage) {
        return new DeferredCommand(
                () -> getCurrentSetTargetArmPositionCommand(targetAngle, targetElevatorPosition, angleSpeedPercentage, elevatorSpeedPercentage),
                Set.of(this)
        );
    }

    private Command getCurrentSetTargetArmPositionCommand(Rotation2d targetAngle, double targetElevatorPosition, double angleSpeedPercentage, double elevatorSpeedPercentage) {
        if (targetElevatorPosition >= getElevatorPositionRevolutions()) {
            return new SequentialCommandGroup(
                    getSetTargetAngleCommand(targetAngle, angleSpeedPercentage),
                    getSetTargetElevatorPositionCommand(targetElevatorPosition, elevatorSpeedPercentage)
            );
        }
        return new SequentialCommandGroup(
                getSetTargetElevatorPositionCommand(targetElevatorPosition, elevatorSpeedPercentage),
                getSetTargetAngleCommand(targetAngle, angleSpeedPercentage)
        );
    }

    private Command getSetTargetAngleCommand(Rotation2d targetAngle, double speedPercentage) {
        return new FunctionalCommand(
                () -> generateAngleMotorProfile(targetAngle, speedPercentage),
                this::setTargetAngleFromProfile,
                (interrupted) -> {
                },
                () -> false,
                this
        );
    }

    private Command getSetTargetElevatorPositionCommand(double targetElevatorPosition, double speedPercentage) {
        return new FunctionalCommand(
                () -> generateElevatorMotorProfile(targetElevatorPosition, speedPercentage),
                this::setTargetElevatorPositionFromProfile,
                (interrupted) -> {
                },
                () -> false,
                this
        );
    }

    private void generateAngleMotorProfile(Rotation2d targetAngle, double speedPercentage) {
        angleMotorProfile = new TrapezoidProfile(
                Conversions.scaleConstraints(ArmConstants.ANGLE_CONSTRAINTS, speedPercentage),
                new TrapezoidProfile.State(targetAngle.getDegrees(), 0),
                new TrapezoidProfile.State(getAnglePositionDegrees().getDegrees(), getAngleVelocityDegreesPerSecond())
        );
        lastAngleMotorProfileGenerationTime = Timer.getFPGATimestamp();
    }

    private void generateElevatorMotorProfile(double targetElevatorPosition, double speedPercentage) {
        elevatorMotorProfile = new TrapezoidProfile(
                Conversions.scaleConstraints(ArmConstants.ELEVATOR_CONSTRAINTS, speedPercentage),
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
        armIO.setAngleMotorPower((calculateAngleMotorVoltage(targetState)));
    }

    private void setTargetElevatorPositionFromProfile() {
        if (elevatorMotorProfile == null) {
            stopElevatorMotors();
            return;
        }

        TrapezoidProfile.State targetState = elevatorMotorProfile.calculate(getElevatorProfileTime());
        armIO.setElevatorMotorPower((calculateElevatorMotorVoltage(targetState)));
    }

    private double getAngleProfileTime() {
        return Timer.getFPGATimestamp() - lastAngleMotorProfileGenerationTime;
    }

    private double getElevatorProfileTime() {
        return Timer.getFPGATimestamp() - lastElevatorMotorProfileGenerationTime;
    }

    private double getElevatorPositionRevolutions() {
        return armInputs.elevatorPositionRevolutions;
    }

    private Rotation2d getAnglePositionDegrees() {
        return Rotation2d.fromRotations(armInputs.angleEncoderPositionSignal.getValue());
    }

    private double getAngleVelocityDegreesPerSecond() {
        return Conversions.perHundredMsToPerSecond(Conversions.revolutionsToDegrees(armInputs.angleEncoderVelocitySignal.getValue()));
    }

    private double getElevatorVelocityRevolutionsPerSecond() {
        return Conversions.perHundredMsToPerSecond(armInputs.elevatorVelocityRevolutions);
    }

    private void stopElevatorMotors() {
        armIO.stopElevatorMotor();
    }

    private void stopAngleMotors() {
        armIO.stopAngleMotor();
    }

    private double calculateAngleMotorVoltage(TrapezoidProfile.State targetState) {
        double pidOutput = armInputs.anglePIDController.calculate(
                armInputs.angleEncoderPositionSignal.getValue(),
                targetState.position
        );
        double feedforward = armInputs.angleFeedforward.calculate(
                Units.degreesToRadians(targetState.position),
                Units.degreesToRadians(targetState.velocity)
        );
        return pidOutput + feedforward;
    }

    private double calculateElevatorMotorVoltage(TrapezoidProfile.State targetState) {
        double pidOutput = armInputs.elevatorPIDController.calculate(
                getElevatorPositionRevolutions(),
                targetState.position
        );
        double feedforward = armInputs.elevatorFeedforward.calculate(targetState.velocity);
        return pidOutput + feedforward;
    }
}

