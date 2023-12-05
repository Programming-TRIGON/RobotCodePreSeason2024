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

import java.lang.annotation.Target;

public class SideShooter extends SubsystemBase {
    private final static SideShooter INSTANCE = new SideShooter();
    private final TalonFX shootingMotor = SideShooterConstants.SHOOTING_MOTOR;
    private final CANSparkMax angleMotor = SideShooterConstants.ANGLE_MOTOR;

    public static SideShooter getInstance() {
        return INSTANCE;
    }

    private Rotation2d getAngleMotorPoison() {
        double positionRevolution = SideShooterConstants.ANGLE_ENCODER_POSITION_SIGNAL.refresh().getValue();
        return Rotation2d.fromRotations(positionRevolution);
    }

    private Double getAngleMotorVelocity() {
        return SideShooterConstants.ANGLE_ENCODER_VELOCITY_SIGNAL.getValue();
    }

    private TrapezoidProfile angleMotorProfile = null;
    private double lastAngleMotorProfileGeneration;

    private double getAngleMotorProfileTime() {
        return Timer.getFPGATimestamp() - lastAngleMotorProfileGeneration;
    }

    private void generateAngleMotorProfile(Rotation2d targetAngle) {
        angleMotorProfile = new TrapezoidProfile(SideShooterConstants.ANGLE_Constraints,
                new TrapezoidProfile.State(targetAngle.getDegrees(), 0),
                new TrapezoidProfile.State(getAngleMotorPoison().getDegrees(), getAngleMotorVelocity()));

        lastAngleMotorProfileGeneration = Timer.getFPGATimestamp();
    }

    private void setTargetAngleMotorProfileTime() {
        if (angleMotorProfile == null) {
            angleMotor.stopMotor();
            return;
        }

        TrapezoidProfile.State targetGetState = angleMotorProfile.calculate(getAngleMotorProfileTime());
        double pidOutPut = SideShooterConstants.ANGLE_PID_CONTROLLER.calculate(
                getAngleMotorPoison().getDegrees(),
                targetGetState.position
        );
        double Feedforward = SideShooterConstants.SIDE_SHOOTER_FEEDFORWARD.calculate(
                Units.degreesToRadians(targetGetState.position),
                Units.degreesToRadians(targetGetState.velocity),
                targetGetState.position
        );


        angleMotor.setVoltage(pidOutPut = Feedforward);
    }
    public Command getSetTargetShooterAngleCommand(Rotation2d angle) {
        return new FunctionalCommand(
                () -> generateAngleMotorProfile(angle),
                this::setTargetAngleMotorProfileTime,
                (interrupted) -> {},
                () -> false,
                this
        );
    }
}




































































































































































































