package frc.trigon.robot.subsystems.arm;


import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.utilities.Conversions;

public class Arm extends SubsystemBase {

    private final CANSparkMax MASTER_ANGLE_MOTOR = ArmConstants.MASTER_ELEVATOR_MOTOR;
    private final CANSparkMax FOLLOWER_ANGLE_MOTOR = ArmConstants.FOLLOWER_ELEVATOR_MOTOR;
    private final static Arm INSTANCE = new Arm();

    private TrapezoidProfile angleMotorProfile = null;
    private double lastAngleMotorProfileGenerationTime;

    public static Arm getInstance() {
        return INSTANCE;
    }

    private Arm(){}

    private void generateAngleMotorProfile(Rotation2d targetAngle){
        angleMotorProfile = new TrapezoidProfile(
                ArmConstants.ANGLE_CONSTRAINTS,
                new TrapezoidProfile.State(targetAngle.getDegrees(),0),
                new TrapezoidProfile.State(getAngleEncoderPosition().getDegrees(), getAngleVelocityDegreesPerSecond())
        );
        lastAngleMotorProfileGenerationTime  = Timer.getFPGATimestamp();
    }

    private void setTargetAngleFromProfile(){
        if (angleMotorProfile == null){
            stopAngleMotors();
            return;
        }
        TrapezoidProfile.State targetState = angleMotorProfile.calculate(getAngleMotorProfileTime());
        MASTER_ANGLE_MOTOR.setVoltage(calculateAngleMotorOutput());
        FOLLOWER_ANGLE_MOTOR.setVoltage(calculateAngleMotorOutput());
    }

    private double calculateAngleMotorOutput(){
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

    private double getAngleMotorProfileTime(){
        return Timer.getFPGATimestamp() - lastAngleMotorProfileGenerationTime;
    }

    private Rotation2d getAngleEncoderPosition(){
        return Rotation2d.fromRotations(ArmConstants.ANGEL_ENCODER_POSITION_SIGNAL.refresh().getValue());
    }

    private double getAngleVelocityDegreesPerSecond(){
        return Conversions.revolutionsToDegrees(ArmConstants.ANGLE_ENCODER_VELOCITY_SIGNAL.refresh().getValue());
    }

    private void stopAngleMotors(){
        MASTER_ANGLE_MOTOR.stopMotor();
        FOLLOWER_ANGLE_MOTOR.stopMotor();
    }
}

