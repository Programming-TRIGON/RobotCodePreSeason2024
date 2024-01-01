package frc.trigon.robot.subsystems.sideshooter;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.subsystems.arm.ArmConstants;
import frc.trigon.robot.utilities.Conversions;

public class SideShooter extends SubsystemBase {
    private final static SideShooter INSTANCE = new SideShooter();
    private final TalonFX shootingMotor = SideShooterConstants.SHOOTING_MOTOR;
    private final CANSparkMax angleMotor = SideShooterConstants.ANGLE_MOTOR;
    private final VoltageOut shootingVoltageRequest = new VoltageOut(0).withEnableFOC(SideShooterConstants.FOC_ENABLED);
    private TrapezoidProfile angleMotorProfile = null;
    private double lastAngleMotorProfileGeneration;

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
                new TrapezoidProfile.State(getAnglePosition().getDegrees(), getAngleVelocityInDegreesPerSeconds())
        );

        lastAngleMotorProfileGeneration = Timer.getFPGATimestamp();
    }

    void setTargetAngleFromProfile() {
        if (angleMotorProfile == null) {
            angleMotor.stopMotor();
            return;
        }

        TrapezoidProfile.State targetState = angleMotorProfile.calculate(getAngleProfileTime());
        angleMotor.setVoltage(calculateAngleOutput(targetState));
    }

    void stopShooting() {
        shootingMotor.stopMotor();
    }

    boolean atAngle(Rotation2d angle) {
        return Math.abs(angle.getDegrees() - getAnglePosition().getDegrees()) < SideShooterConstants.ANGLE_TOLERANCE;
    }

    private double calculateAngleOutput(TrapezoidProfile.State targetState) {
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
        double positionRevolutions = SideShooterConstants.ANGLE_ENCODER_POSITION_SIGNAL.refresh().getValue();
        return Rotation2d.fromRotations(positionRevolutions);
    }

    private double getAngleVelocityInDegreesPerSeconds() {
        return Conversions.revolutionsToDegrees(SideShooterConstants.ANGLE_ENCODER_VELOCITY_SIGNAL.refresh().getValue());
    }

    private double getAngleProfileTime() {
        return Timer.getFPGATimestamp() - lastAngleMotorProfileGeneration;
    }
}