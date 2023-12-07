package frc.trigon.robot.subsystems.turret;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Pose2d;
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

    public Command getSetTurretPositionCommand() {
        return new FunctionalCommand(
                () -> {
                },
                this::turretCalculate,
                (interrupted) -> stop(),
                () -> false,
                this
        );
    }

    private void turretCalculate() {
        if (atTarget())
            return;
        double xDifference = hubPose.getX() - getPosition();
        double yDifference = hubPose.getY() - getPosition();
        double desiredHeading = Math.atan2(xDifference, yDifference);
        if (checkTurretSpinLimit(desiredHeading))
            return;
        setMotorVoltage(desiredHeading);
    }

    private boolean atTarget() {
        return getPosition() == hubPose.getRotation().getRotations();
    }

    private boolean checkTurretSpinLimit(double angleChange) {
        return Math.abs(getPosition() - angleChange) == 200;
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
