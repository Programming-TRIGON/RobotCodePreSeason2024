package frc.trigon.robot.subsystems.turret;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
    private final static Turret INSTANCE = new Turret();
    private final TalonFX motor = TurretConstants.MOTOR;
    private final Pose2d
            hubPose = TurretConstants.HUB_POSE,
            robotPose = TurretConstants.ROBOT_POSE;
    private final VoltageOut voltageRequest = new VoltageOut(0, TurretConstants.FOC_ENABLED, false);

    public static Turret getInstance() {
        return INSTANCE;
    }

    private Turret() {
    }

    public Command getSetMotorOutputCommand(Pose2d robotPose) {
        return new FunctionalCommand(
                () -> {
                },
                () -> setMotorOutput(robotPose),
                (interrupted) -> stop(),
                () -> false,
                this
        );
    }

    private void setMotorOutput(Pose2d robotPose) {
        setMotorVoltage(calculateMotorOutput(robotPose));
    }

    private double calculateMotorOutput(Pose2d robotPose) {
        double targetAngle = calculateTargetAngle(robotPose);
        double targetAngleAfterLimitCheck = limitCheck(targetAngle);
        double error = calculateError(targetAngleAfterLimitCheck);
        double output = TurretConstants.PID_CONTROLLER.calculate(error);
        return output;
    }

    private double calculateTargetAngle(Pose2d robotPose) {
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotHeading = robotPose.getRotation().getRadians();
        double hubVecX = hubPose.getX() - robotX;
        double hubVecY = hubPose.getY() - robotY;
        double theta = Math.atan2(hubVecY, hubVecX);
        double targetAngle = theta - robotHeading;
        return targetAngle;
    }

    private double limitCheck(double targetAngle) {
        if (targetAngle > 200) {
            targetAngle -= 360;
        } else if (targetAngle < -200) {
            targetAngle += 360;
        }
        return targetAngle;
    }

    private double calculateError(double targetAngle) {
        double currentAngleDegrees = Units.rotationsToDegrees(getPosition());
        double error = targetAngle - currentAngleDegrees;
        return error;
    }

    private double getPosition() {
        return TurretConstants.STATUS_SIGNAL.refresh().getValue();
    }

    private void setMotorVoltage(double voltage) {
        motor.setControl(voltageRequest.withOutput(voltage));
    }

    private void stop() {
        motor.stopMotor();
    }
}
