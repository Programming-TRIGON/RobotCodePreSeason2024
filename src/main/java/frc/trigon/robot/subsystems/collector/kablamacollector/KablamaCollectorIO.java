package frc.trigon.robot.subsystems.collector.kablamacollector;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.trigon.robot.subsystems.collector.CollectorIO;

public class KablamaCollectorIO extends CollectorIO {
    private final TalonSRX motor = KablamaCollectorConstants.MOTOR;

    @Override
    protected void setPower(double power) {
        motor.set(ControlMode.PercentOutput, power);
    }
    @Override
    protected void stop() {
        motor.set(ControlMode.Disabled, 0);
    }
}
