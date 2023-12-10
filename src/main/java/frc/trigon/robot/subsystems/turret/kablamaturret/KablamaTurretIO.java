package frc.trigon.robot.subsystems.turret.kablamaturret;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.subsystems.turret.TurretIO;
import frc.trigon.robot.subsystems.turret.TurretInputsAutoLogged;

public class KablamaTurretIO extends TurretIO {
    private final TalonFX motor = KablamaTurretConstants.MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0, KablamaTurretConstants.FOC_ENABLED, false);
    private final Pose2d hubPose = KablamaTurretConstants.HUB_POSE;

    @Override
    protected void updateInputs(TurretInputsAutoLogged inputs) {
        inputs.motorPositionDegrees = getPositionDegrees();
    }

    @Override
    protected void stop() {
        motor.stopMotor();
    }

    @Override
    protected void calculateMotorOutput(Pose2d robotPose) {
        double targetAngleRadians = calculateTargetAngle(robotPose);
        double targetAngleAfterLimitCheck = limitCheck(Units.radiansToDegrees(targetAngleRadians));
        double error = calculateError(targetAngleAfterLimitCheck);
        double output = KablamaTurretConstants.PID_CONTROLLER.calculate(error);
        setMotorVoltage(output);
    }

    private double calculateTargetAngle(Pose2d robotPose) {
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotHeading = robotPose.getRotation().getRadians();
        double differenceX = hubPose.getX() - robotX;
        double differenceY = hubPose.getY() - robotY;
        double theta = Math.atan2(differenceY, differenceX);
        double targetAngle = theta - robotHeading;
        return targetAngle;
    }

    private double limitCheck(double targetAngleDegrees) {
        if (targetAngleDegrees > 200) {
            targetAngleDegrees -= 360;
        } else if (targetAngleDegrees < -200) {
            targetAngleDegrees += 360;
        }
        return targetAngleDegrees;
    }

    private double calculateError(double targetAngleDegrees) {
        double error = targetAngleDegrees - getPositionDegrees();
        return error;
    }

    private double getPositionDegrees() {
        return Units.rotationsToDegrees(KablamaTurretConstants.STATUS_SIGNAL.refresh().getValue());
    }

    private void setMotorVoltage(double voltage) {
        motor.setControl(voltageRequest.withOutput(voltage));
    }
}
