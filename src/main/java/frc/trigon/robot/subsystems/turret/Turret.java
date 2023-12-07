package frc.trigon.robot.subsystems.turret;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

public class Turret extends SubsystemBase {
    private final static Turret INSTANCE = new Turret();
    private final TalonFX motor = TurretConstants.MOTOR;
    private final Pose2d pose = TurretConstants.POSE;
    private final VoltageOut voltageRequest = new VoltageOut(0, TurretConstants.FOC_ENABLED, false);

    public static Turret getInstance() {
        return INSTANCE;
    }

    private Turret() {
    }

    public Command getSetTurretPositionCommand(Supplier<Double> targetAngle) {
        return new FunctionalCommand(
                () -> {
                },
                () -> turretCalculate(targetAngle.get()),
                (interrupted) -> stop(),
                () -> false,
                this
        );
    }

    private void turretCalculate(double targetAngle) {
        if (atTarget())
            return;
        if (!checkTurretSpinLimit(targetAngle))
            return;
        double output = TurretConstants.PID_CONTROLLER.calculate(targetAngle);
        setMotorVoltage(output);
    }

    private boolean atTarget() {
        return getPosition() == TurretConstants.TARGET_POSITION.getAngle().getRotations();
    }

    private boolean checkTurretSpinLimit(double targetAngle) {
        return Math.abs(getPosition() - targetAngle) == 200;
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
