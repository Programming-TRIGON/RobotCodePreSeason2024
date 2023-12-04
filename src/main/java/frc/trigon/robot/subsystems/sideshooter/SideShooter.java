package frc.trigon.robot.subsystems.sideshooter;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SideShooter extends SubsystemBase {
    private final static SideShooter INSTANCE = new SideShooter();
    private final TalonFX shootingMotor = SideShooterConstants.SHOOTING_MOTOR;
    private final CANSparkMax angelMotor = SideShooterConstants.ANGEL_MOTOR;
    private final VoltageOut shootingVoltageRequest = new VoltageOut(0, SideShooterConstants.FOC_ENABLED, false);
    private TrapezoidProfile angleMotorProfile = null;
    private double lastAngleMotorProfileGeneration;
    private double angleTolerance = 1;

    public static SideShooter getInstance() {
        return INSTANCE;
    }

    private SideShooter() {
    }

    void setTargetShootingVoltage(double targetVoltage) {
        shootingMotor.setControl(shootingVoltageRequest.withOutput(targetVoltage));
    }

    void generateAngleMotorProfile(Rotation2d targetAngle) {
        angleMotorProfile = new TrapezoidProfile(
                SideShooterConstants.ANGLE_CONSTRAINTS,
                new TrapezoidProfile.State(targetAngle.getDegrees(), 0),
                new TrapezoidProfile.State(getAnglePosition().getDegrees(), getAngleVelocity())
        );

        lastAngleMotorProfileGeneration = Timer.getFPGATimestamp();
    }

    void setTargetAngleFromProfile() {
        if (angleMotorProfile == null) {
            angelMotor.stopMotor();
            return;
        }

        TrapezoidProfile.State targetState = angleMotorProfile.calculate(getAngleProfileTime());
        double output = SideShooterConstants.ANGLE_PID_CONTROLLER.calculate(getAnglePosition().getDegrees(), targetState.position);
        angelMotor.setVoltage(output);
    }

    void stopShooting() {
        shootingMotor.stopMotor();
    }

    boolean atAngle(Rotation2d angle){
        return  Math.abs(angle.getDegrees() - getAnglePosition().getDegrees()) < angleTolerance;
    }

    private Rotation2d getAnglePosition() {
        double positionRevolutions = SideShooterConstants.ANGEL_ENCODER_POSITION_SIGNAL.refresh().getValue();
        return Rotation2d.fromRotations(positionRevolutions);
    }

    private double getAngleVelocity() {
        return SideShooterConstants.ANGEL_ENCODER_VELOCITY_SIGNAL.refresh().getValue();
    }

    private double getAngleProfileTime() {
        return Timer.getFPGATimestamp() - lastAngleMotorProfileGeneration;
    }
}