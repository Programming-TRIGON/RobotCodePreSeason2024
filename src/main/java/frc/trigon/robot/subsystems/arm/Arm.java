package frc.trigon.robot.subsystems.arm;


import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

    private final static Arm INSTANCE = new Arm();

    public static Arm getInstance() {
        return INSTANCE;
    }

    private Arm() {
    }

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

    private void generateAngleMotorProfile(Rotation2d targetAngle){
        angleMotorProfile = new TrapezoidProfile(
                ArmConstants.ANGLE_CONSTRAINTS,
                new TrapezoidProfile.State(targetAngle.getDegrees(),0),
                new TrapezoidProfile.State(getAngleMotorPositionDegrees(), getAngleMotorVelocity())
        );

        lastAngleMotorProfileGenerationTime = Timer.getFPGATimestamp();
    }

    private double getAngleMotorPositionDegrees(){
        return Rotation2d.fromDegrees(angleEncoder.getPosition().getValue()).getDegrees();
    }

    private double getAngleMotorVelocity(){
        return angleEncoder.getVelocity().getValue();
    }

}

