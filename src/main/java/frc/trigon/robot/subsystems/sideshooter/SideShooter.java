package frc.trigon.robot.subsystems.sideshooter;


import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.utilities.Conversions;

public class SideShooter extends SubsystemBase {
    private final TalonFX shootingMotor = SideShooterConstants.SHOOTING_MOTOR;
    private final CANSparkMax angleMotor = SideShooterConstants.ANGLE_MOTOR;
    private TrapezoidProfile angleMotorProfile = null;
    private double lastAngleMotorProfileGenerationTime;

    private final static SideShooter INSTANCE = new SideShooter();

    public static SideShooter getInstance() {
        return INSTANCE;
    }

    double getAnglePosition() {
        double positionRevolutions = SideShooterConstants.ANGLE_ENCODER_POSITION_SIGNAL.refresh().getValue();
        return positionRevolutions;
    }

    Double getAngleVelocityDegreesPerSecond() {
        return Conversions.revolutionsToDegrees(SideShooterConstants.ANGLE_ENCODER_VELOCITY_SIGNAL.refresh().getValue());
    }

    double getAngleMotorProfileTime() {
        return Timer.getFPGATimestamp() - lastAngleMotorProfileGenerationTime;
    }

    void generateAngleMotorProfile(Rotation2d targetAngle) {
        angleMotorProfile = new TrapezoidProfile(SideShooterConstants.ANGLE_Constraints,
                new TrapezoidProfile.State(targetAngle.getDegrees(), 0),
                new TrapezoidProfile.State(getAnglePosition(), getAngleVelocityDegreesPerSecond()));

        lastAngleMotorProfileGenerationTime = Timer.getFPGATimestamp();
    }

    double calculateAngleOutput(TrapezoidProfile.State targetState) {
        double pidOutPut = SideShooterConstants.ANGLE_PID_CONTROLLER.calculate(
                getAnglePosition(),
                targetState.position
        );
        double feedforward = SideShooterConstants.SIDE_SHOOTER_FEEDFORWARD.calculate(
                Units.degreesToRadians(targetState.position),
                targetState.velocity
        );
        angleMotor.setVoltage(pidOutPut = feedforward);
        return feedforward + pidOutPut;
    }

    void setTargetAngleMotorProfileTime() {
        if (angleMotorProfile == null) {
            angleMotor.stopMotor();
            return;
        }
        TrapezoidProfile.State targetState = angleMotorProfile.calculate(getAngleMotorProfileTime());
        angleMotor.setVoltage(calculateAngleOutput(targetState));
    }
}
