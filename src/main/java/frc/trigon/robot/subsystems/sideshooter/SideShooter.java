package frc.trigon.robot.subsystems.sideshooter;


import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.utilities.Conversions;

public class SideShooter extends SubsystemBase {
    private final static SideShooter INSTANCE = new SideShooter();

    private final TalonFX SHOOTING_MOTOR = SideShooterConstants.SHOOTING_MOTOR;
    private final CANSparkMax ANGLE_MOTOR = SideShooterConstants.ANGLE_MOTOR;
    private final CANcoder ANGLE_ENCODER = SideShooterConstants.ANGLE_ENCODER;

    private TrapezoidProfile angleMotorProfile = null;
    private double lastAngleMotorProfileGenerationTime;

    public static SideShooter getInstance() {
        return INSTANCE;
    }

    private SideShooter() {
    }

    private void generateAngleMotorProfile(Rotation2d targetAngle) {
        angleMotorProfile = new TrapezoidProfile(
                SideShooterConstants.ANGLE_CONSTRAINTS,
                new TrapezoidProfile.State(targetAngle.getDegrees(), 0),
                new TrapezoidProfile.State(getAnglePosition().getDegrees(), getAngleVelocityDegreesPerSecond())
        );
        lastAngleMotorProfileGenerationTime = Timer.getFPGATimestamp();
    }

    private Rotation2d getAnglePosition() {
        return Rotation2d.fromRotations(SideShooterConstants.ANGLE_ENCODER_POSITION_SIGNAL.refresh().getValue());
    }
    private double getAngleVelocityDegreesPerSecond(){
        return Conversions.perHundredMsToPerSecond(Conversions.revolutionsToDegrees(SideShooterConstants.ANGLE_ENCODER_VELOCITY_SIGNAL.refresh().getValue()));
    }

    private double getAngleMotorProfileTime(){
        return Timer.getFPGATimestamp() - lastAngleMotorProfileGenerationTime;
    }
    private void setTargetAngleFromProfile(){
        if (angleMotorProfile==null){
            stop_angle_motor();
        }
        TrapezoidProfile.State targetState = angleMotorProfile.calculate(getAngleMotorProfileTime());
        calculateAngleMotorOutput();
    }
    public Command getSetTargetShooterAngleCommand(Rotation2d angle){
     return new FunctionalCommand(
             ()->generateAngleMotorProfile(angle),
             this::setTargetAngleFromProfile,
             (interrupted)->{},
             ()->false,
             this
     );
    }

    private void calculateAngleMotorOutput(){
        TrapezoidProfile.State targetState = angleMotorProfile.calculate(getAngleMotorProfileTime());
        double pidOutput = SideShooterConstants.ANGLE_PID_CONTROLLER.calculate(
                getAnglePosition().getDegrees(),
                targetState.position
        );
        double feedforward = SideShooterConstants.ANGLE_MOTOR_FEEDFORWARD.calculate(
                Units.degreesToRadians(targetState.position),
                Units.degreesToRadians(targetState.velocity)
        );
        ANGLE_MOTOR.setVoltage(pidOutput + feedforward);
    }
    private void stop_angle_motor(){
        ANGLE_MOTOR.stopMotor();
    }
}

