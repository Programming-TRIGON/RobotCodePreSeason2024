package frc.trigon.robot.subsystems.collector.simulationcollector;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.trigon.robot.subsystems.collector.CollectorIO;
import frc.trigon.robot.subsystems.collector.CollectorInputsAutoLogged;

public class SimulationCollectorIO extends CollectorIO {
    private final TalonSRX motor = SimulationCollectorIOConstants.MOTOR;
    private final DCMotor dcMotor = SimulationCollectorIOConstants.DC_MOTOR;
    private final DCMotorSim dcMotorSim =
            new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(dcMotor, 1, 1),
                    dcMotor,
                    1
            );
    public SimulationCollectorIO() {
        System.out.println("[Init] Creating CollectorIOSim");
    }

    @Override
    protected void updateInputs(CollectorInputsAutoLogged inputs) {
        inputs.motorVoltage = motor.getMotorOutputVoltage();
        inputs.motorCurrent = motor.getSupplyCurrent();
    }

    @Override
    protected void setPower(double power) {
        motor.set(ControlMode.PercentOutput, power);
    }

    @Override
    protected void stop() {
        motor.set(ControlMode.Disabled, 0);
    }
}
