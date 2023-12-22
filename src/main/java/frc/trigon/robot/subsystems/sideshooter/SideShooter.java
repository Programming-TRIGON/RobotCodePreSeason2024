package frc.trigon.robot.subsystems.sideshooter;


import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.utilities.Conversions;

import java.lang.annotation.Target;

public class SideShooter extends SubsystemBase {
    private final TalonFX shootingMotor = SideShooterConstants.SHOOTING_MOTOR;
    private final CANSparkMax angleMotor = SideShooterConstants.ANGLE_MOTOR;
    private TrapezoidProfile angleMotorProfile = null;
    private double lastAngleMotorProfileGeneration;

    public static SideShooter getInstance() {
        return INSTANCE;
    }
    private final static SideShooter INSTANCE = new SideShooter();

    public Command getSetTargetShooterAngleCommand(Rotation2d targetAngle) {
        return new FunctionalCommand(
                () -> generateAngleMotorProfile(targetAngle),
                this::setTargetAngleMotorProfileTime,
                (interrupted) -> {
                },
                () -> false,
                this
        );
    }

    private Rotation2d getAngleMotorPosition() {
        double positionRevolutions = SideShooterConstants.ANGLE_ENCODER_POSITION_SIGNAL.refresh().getValue();
        return Rotation2d.fromRotations(positionRevolutions);
    }

    private Double getAngleMotorVelocityRotationsPerSecond() {

        return Conversions.revolutionsToDegrees();
    }

    private double getAngleMotorProfileTime() {
        return Timer.getFPGATimestamp() - lastAngleMotorProfileGeneration;
    }

    private void generateAngleMotorProfile(Rotation2d targetAngle) {
        angleMotorProfile = new TrapezoidProfile(SideShooterConstants.ANGLE_Constraints,
                new TrapezoidProfile.State(targetAngle.getDegrees(), 0),
                new TrapezoidProfile.State(getAngleMotorPosition().getDegrees(), getAngleMotorVelocityRotationsPerSecond()));

        lastAngleMotorProfileGeneration = Timer.getFPGATimestamp();
    }

    private void calculateAngleOutput{
    TrapezoidProfile.State targetState = angleMotorProfile.calculate(getAngleMotorProfileTime());
    double pidOutPut = SideShooterConstants.ANGLE_PID_CONTROLLER.calculate(
            getAngleMotorPosition().getDegrees(),
            targetState.position
    );
    double Feedforward = SideShooterConstants.SIDE_SHOOTER_FEEDFORWARD.calculate(
            Units.degreesToRadians(targetState.position),
            Units.degreesToRadians(targetState.velocity),
            targetState.position
    );
    }

    private void setTargetAngleMotorProfileTime() {
        if (angleMotorProfile == null) {
            angleMotor.stopMotor();
            return;
        }


    }
}
