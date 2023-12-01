package frc.trigon.robot.subsystems.sideshooter;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.utilities.Conversions;

public class SideShooter extends SubsystemBase {
    private final static SideShooter INSTANCE = new SideShooter();

    private final TalonFX shootingMotor = SideShooterConstants.SHOOTING_MOTOR;
    private final CANSparkMax angleMotor = SideShooterConstants.ANGLE_MOTOR;
    private final CANcoder angleEncoder = SideShooterConstants.ANGLE_ENCODER;

    private final VoltageOut shootingVoltageRequest = new VoltageOut(0, SideShooterConstants.FOC_ENABLE, false);

    private TrapezoidProfile angleMotorProfile = null;
    private double lastAngleMotorProfileGenerationTime;

    private SideShooter() {
    }

    public static SideShooter getInstance() {
        return INSTANCE;
    }

    private Command getSetTargetStateCommand(boolean byOrder, Rotation2d targetAngle, double targetVoltage){
        if (!byOrder){
            return new ParallelCommandGroup(
                    getSetTargetShooterAngleCommand(targetAngle),
                    getSetTargetShootingVoltageCommand(targetVoltage)
            );
        }
        else{
            return new SequentialCommandGroup(
                    getSetTargetShooterAngleCommand(targetAngle),
                    getSetTargetShootingVoltageCommand(targetVoltage)
            );
        }
    }

    public Command getSetTargetShooterAngleCommand(Rotation2d targetAngle) {
        return new FunctionalCommand(
                () -> generateAngleMotorProfile(targetAngle),
                this::setTargetAngleFromProfile,
                (interrupted) -> {
                },
                () -> false,
                this
        );
    }

    public Command getSetTargetShootingVoltageCommand(double targetVoltage) {
        return new StartEndCommand(
                () -> setTargetShootingVoltage(targetVoltage),
                this::stopAngleMotor,
                this
        );
    }

    private void generateAngleMotorProfile(Rotation2d targetAngle) {
        angleMotorProfile = new TrapezoidProfile(
                SideShooterConstants.ANGLE_CONSTRAINTS,
                new TrapezoidProfile.State(targetAngle.getDegrees(), 0),
                new TrapezoidProfile.State(getAnglePosition().getDegrees(), getAngleVelocityDegreesPerSecond())
        );
        lastAngleMotorProfileGenerationTime = Timer.getFPGATimestamp();
    }

    private void setTargetAngleFromProfile() {
        if (angleMotorProfile == null) {
            stopAngleMotor();
            return;
        }

        TrapezoidProfile.State targetState = angleMotorProfile.calculate(getAngleMotorProfileTime());
        angleMotor.setVoltage(calculateAngleMotorOutput());
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

    private void setTargetShootingVoltage(double targetVoltage) {
        shootingMotor.setControl(shootingVoltageRequest.withOutput(targetVoltage));
    }

    private void stopAngleMotor() {
        angleMotor.stopMotor();
    }
}

