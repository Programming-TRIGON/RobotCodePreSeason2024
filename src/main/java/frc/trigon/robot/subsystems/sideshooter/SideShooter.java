package frc.trigon.robot.subsystems.sideshooter;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.utilities.Conversions;

public class SideShooter extends SubsystemBase {
    private final static SideShooter INSTANCE = new SideShooter();
    private final TalonFX shootingMotor = SideShooterConstants.SHOOTING_MOTOR;
    private final CANSparkMax angleMotor = SideShooterConstants.ANGLE_MOTOR;
    private final CANcoder angleEncoder = SideShooterConstants.ANGLE_ENCODER;

    private static final VoltageOut driveVoltageRequest = new VoltageOut(0).withEnableFOC(SideShooterConstants.FOC_ENABLED);
    private TrapezoidProfile
            angleMotorProfile = null;
    private double lastAngleMotorProfileGenerationTime;

    public static SideShooter getInstance() {
        return INSTANCE;
    }

    private SideShooter() {
    }

    boolean atAngle(Rotation2d atAngle){
        return Math.abs(atAngle.getDegrees() - getAnglePosition().getDegrees()) <= 1;
    }

    void generateAngleMotorProfile(Rotation2d targetAngle) {
        angleMotorProfile = new TrapezoidProfile(
                SideShooterConstants.ANGLE_CONSTRAINTS,
                new TrapezoidProfile.State(targetAngle.getDegrees(), 0),
                new TrapezoidProfile.State(getAnglePosition().getDegrees(), getAngleVelocityDegreesPerSecond())
        );
        lastAngleMotorProfileGenerationTime  = Timer.getFPGATimestamp();
    }

    void setTargetAngleFromProfile() {
        if (angleMotorProfile == null) {
            stopAngleMotor();
            return;
        }

        TrapezoidProfile.State targetState = angleMotorProfile.calculate(getAngleMotorProfileTime());
        angleMotor.setVoltage(calculateAngleMotorOutput());
    }

    void setTargetShootingVoltage(double targetVoltage) {
        shootingMotor.setControl(driveVoltageRequest.withOutput(targetVoltage));
    }

    void stopAngleMotor() {
        angleMotor.stopMotor();
    }

    private double calculateAngleMotorOutput() {
        TrapezoidProfile.State targetState = angleMotorProfile.calculate(getAngleMotorProfileTime());
        double pidOutput = SideShooterConstants.ANGLE_PID_CONTROLLER.calculate(
                getAnglePosition().getDegrees(),
                targetState.position
        );
        double feedforward = SideShooterConstants.ANGLE_MOTOR_FEEDFORWARD.calculate(
                Units.degreesToRadians(targetState.position),
                Units.degreesToRadians(targetState.velocity)
        );
        return pidOutput + feedforward;
    }

    private Rotation2d getAnglePosition() {
        return Rotation2d.fromRotations(SideShooterConstants.ANGLE_ENCODER_POSITION_SIGNAL.refresh().getValue());
    }

    private double getAngleVelocityDegreesPerSecond() {
        return Conversions.revolutionsToDegrees(SideShooterConstants.ANGLE_ENCODER_VELOCITY_SIGNAL.refresh().getValue());
    }

    private double getAngleMotorProfileTime() {
        return Timer.getFPGATimestamp() - lastAngleMotorProfileGenerationTime;
    }
}

