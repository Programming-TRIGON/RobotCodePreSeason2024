package frc.trigon.robot.subsystems.turret;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.utilities.Conversions;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
    private final static Turret INSTANCE = new Turret();
    private final TurretIO turretIO = TurretIO.generateIO();
    private final TurretInputsAutoLogged turretInputs = new TurretInputsAutoLogged();

    private TrapezoidProfile motorProfile = null;
    private double lastMotorProfileGenerationTime;

    public static Turret getInstance() {
        return INSTANCE;
    }

    private Turret() {
    }

    @Override
    public void periodic() {
        turretIO.updateInputs(turretInputs);
        Logger.processInputs("Turret", turretInputs);
        updateMechanism();
    }

    void generateMotorProfile(Rotation2d targetAngle, double speedPercentage) {
        motorProfile = new TrapezoidProfile(
                Conversions.scaleConstraints(TurretConstants.MOTOR_CONSTRAINTS, speedPercentage),
                new TrapezoidProfile.State(targetAngle.getDegrees(), 0),
                new TrapezoidProfile.State(turretInputs.motorAngleDegrees, turretInputs.motorVelocityDegreesPerSecond)
        );
        lastMotorProfileGenerationTime = Timer.getFPGATimestamp();
    }

    void setTargetAngleFromProfile() {
        if (motorProfile == null) {
            turretIO.stopMotor();
            return;
        }

        TrapezoidProfile.State targetState = motorProfile.calculate(getMotorProfileTime());
        TurretConstants.TARGET_TURRET_POSITION_LIGAMENT.setAngle(targetState.position);
        turretIO.setTargetAngleState(targetState);
    }

    Rotation2d calculateAngleToHub(Pose2d robotPosition) {
        double
                y = Math.abs(robotPosition.getY() - TurretConstants.HUB_POSITION.getY()),
                x = Math.abs(robotPosition.getX() - TurretConstants.HUB_POSITION.getX());
        return Rotation2d.fromDegrees(Math.tan(y / x));
    }

    private double getMotorProfileTime() {
        return Timer.getFPGATimestamp() - lastMotorProfileGenerationTime;
    }

    private void updateMechanism() {
        TurretConstants.TURRET_LIGAMENT.setAngle(turretInputs.motorAngleDegrees);
    }
}

