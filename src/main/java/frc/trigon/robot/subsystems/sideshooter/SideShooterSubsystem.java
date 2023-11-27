package frc.trigon.robot.subsystems.sideshooter;


import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SideShooterSubsystem extends SubsystemBase {
    private final static SideShooterSubsystem INSTANCE = new SideShooterSubsystem();
    private final static TalonFX SHOOTER_MOTOR = SideShooterConstants.SHOOTING_MOTOR;
    private final static CANSparkMax ANGLE_MOTOR = SideShooterConstants.ANGEL_MOTOR;
    private final static CANcoder ANGLE_ENCODER = SideShooterConstants.ANGLE_ENCODER;
    private final VoltageOut ShootingVoltageRequest = new VoltageOut(0, SideShooterConstants.FOC_ENABLED, false);
    public static SideShooterSubsystem getInstance() {
        return INSTANCE;
    }
    private TrapezoidProfile angleMotorProfile = null;
    private double lastAngleMotorProfileGeneration;

    private SideShooterSubsystem() {
    }

    private void setTargetShootingVoltage(double voltage) {
        SHOOTER_MOTOR.setControl(ShootingVoltageRequest.withOutput(voltage));
    }

    private Rotation2d getAngleMotorPosition(){
        double positionRevolutions = SideShooterConstants.ANGEL_ENCODER_POSITION_SIGNAL.refresh().getValue();
        return Rotation2d.fromRotations(positionRevolutions);
    }

    private double getAngleMotorVelocity(){
        return SideShooterConstants.ANGEL_ENCODER_VELOCITY_SIGNAL.refresh().getValue();
    }

    private double getAngleMotorProfileTime(){
        return Timer.getFPGATimestamp() - lastAngleMotorProfileGeneration;
    }

    private void generateAngleMotorProfile(Rotation2d targetAngle){
        angleMotorProfile = new TrapezoidProfile(
                SideShooterConstants.ANGLE_CONSTRAINTS,
                new TrapezoidProfile.State(targetAngle.getDegrees(), 0),
                new TrapezoidProfile.State(getAngleMotorPosition().getDegrees(), getAngleMotorVelocity())
        );

        lastAngleMotorProfileGeneration = Timer.getFPGATimestamp();
    }

    private void setTargetAngleFromProfile(){
        if (angleMotorProfile == null){
            ANGLE_MOTOR.stopMotor();
            return;
        }

        TrapezoidProfile.State targetState = angleMotorProfile.calculate(getAngleMotorProfileTime());
        SideShooterConstants.ANGLE_PID_CONTROLLER.calculate(getAngleMotorPosition().getDegrees(), targetState.position);
    }

    public Command getSetTargetAngleCommand(Rotation2d targetAngle){
        return new FunctionalCommand(
                ()-> generateAngleMotorProfile(targetAngle),
                this::setTargetAngleFromProfile,
                (interrupted) -> {
                },
                ()-> false,
                this
        );
    }
}

