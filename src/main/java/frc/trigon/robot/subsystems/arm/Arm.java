package frc.trigon.robot.subsystems.arm;


import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

    public Command getSetTargetAngleCommand(Rotation2d angle){
        return new FunctionalCommand(
                () -> generateAngleMotorProfile(angle),
                this::setTargetAngleFromProfile,
                (interrupted) -> {},
                () -> false,
                this
        );
    }

    public Command getSetElevatorTargetCommand(double rotations){
        return new FunctionalCommand(
                () -> generateElevatorMotorProfile(rotations),
                this::setTargetElevatorFromProfile,
                (interrupted) -> {},
                () -> false,
                this
        );
    }

    private void setTargetAngleFromProfile(){
        if (angleMotorProfile == null){
            stopAngleMotors();
            return;
        }

        final TrapezoidProfile.State targetState = angleMotorProfile.calculate(getAngleMotorProfileTime());
        masterAngleMotor.getPIDController().setReference(targetState.position, CANSparkMax.ControlType.kPosition);
    }

    private void setTargetElevatorFromProfile(){
        if (elevatorMotorProfile == null){
            stopElevatorMotors();
            return;
        }

        TrapezoidProfile.State targetState = elevatorMotorProfile.calculate(getElevatorMotorProfileTime());
        masterElevatorMotor.getPIDController().setReference(targetState.position, CANSparkMax.ControlType.kPosition);
    }

    private void generateAngleMotorProfile(Rotation2d targetAngle){
        angleMotorProfile = new TrapezoidProfile(
                ArmConstants.ANGLE_CONSTRAINTS,
                new TrapezoidProfile.State(targetAngle.getDegrees(),0),
                new TrapezoidProfile.State(getAngleMotorPosition().getDegrees(), getAngleMotorVelocityRevolutionsPerSecond())
        );

        lastAngleMotorProfileGenerationTime = Timer.getFPGATimestamp();
    }

    private void generateElevatorMotorProfile(double targetPostition){
        angleMotorProfile = new TrapezoidProfile(
                ArmConstants.ELEVATOR_CONSTRAINTS,
                new TrapezoidProfile.State(targetPostition,0),
                new TrapezoidProfile.State(getElevatorMotorPositionRevolutions(), getElevatorVelocityRevolutionsPerSecond())
        );

        lastElevatorMotorProfileGenerationTime = Timer.getFPGATimestamp();
    }

    private double getAngleMotorProfileTime(){
        return Timer.getFPGATimestamp() - lastAngleMotorProfileGenerationTime;
    }

    private double getElevatorMotorProfileTime(){
        return Timer.getFPGATimestamp() - lastElevatorMotorProfileGenerationTime;
    }

    private Rotation2d getAngleMotorPosition (){
        return Rotation2d.fromRotations(angleEncoder.getPosition().getValue());
    }

    private double getElevatorMotorPositionRevolutions(){
        return Conversions.magTicksToRevolutions(elevatorEncoder.getSelectedSensorPosition());
    }

    private double getAngleMotorVelocityRevolutionsPerSecond(){
        return angleEncoder.getVelocity().getValue();
    }

    private double getElevatorVelocityRevolutionsPerSecond(){
        return Conversions.magTicksToRevolutions(elevatorEncoder.getSelectedSensorVelocity());
    }

    private void stopAngleMotors(){
        masterAngleMotor.stopMotor();
        followerAngleMotor.stopMotor();
    }

    private void stopElevatorMotors(){
        masterElevatorMotor.stopMotor();
        followerElevatorMotor.stopMotor();
    }
}

