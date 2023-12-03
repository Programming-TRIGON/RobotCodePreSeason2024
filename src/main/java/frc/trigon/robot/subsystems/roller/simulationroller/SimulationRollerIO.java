package frc.trigon.robot.subsystems.roller.simulationroller;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.roller.RollerIO;
import frc.trigon.robot.subsystems.roller.RollerInputsAutoLogged;
import frc.trigon.robot.subsystems.roller.toohardroller.ToohardRollerConstants;

public class SimulationRollerIO extends RollerIO {
    private final SingleJointedArmSim angleSimulation = SimulationRollerConstants.ANGLE_MOTOR;
    private final DCMotorSim collectorSimulation = SimulationRollerConstants.COLLECTOR_MOTOR;

    private double angleMotorVoltage, collectorMotorVoltage;

    @Override
    protected void updateInputs(RollerInputsAutoLogged inputs) {
        angleSimulation.update(RobotConstants.PERIODIC_TIME_SECONDS);
        collectorSimulation.update(RobotConstants.PERIODIC_TIME_SECONDS);

        inputs.angleMotorVoltage = angleMotorVoltage;
        inputs.angleMotorCurrent = angleSimulation.getCurrentDrawAmps();
        inputs.collectionMotorVoltage = collectorMotorVoltage;
        inputs.collectionMotorCurrent = collectorSimulation.getCurrentDrawAmps();
        inputs.forwardLimitSwitch = angleSimulation.hasHitUpperLimit();
        inputs.backwardLimitSwitch = angleSimulation.hasHitLowerLimit();
    }

    @Override
    protected void setAngleMotorPower(double power) {
        angleSimulation.setInputVoltage(getAngleVoltageFromPower(power));
    }

    @Override
    protected void setCollectionMotorPower(double power) {
        collectorSimulation.setInputVoltage(getCollectorMotorVoltageFromPower(power));
    }

    @Override
    protected void stopAngleMotor() {
        angleSimulation.setInputVoltage(0);
    }

    @Override
    protected void stopCollectionMotor() {
        collectorSimulation.setInputVoltage(0);
    }

    private double getAngleVoltageFromPower(double power) {
        return power * ToohardRollerConstants.ANGLE_VOLTAGE_COMPENSATION_SATURATION;
    }

    private double getCollectorMotorVoltageFromPower(double power) {
        return power * ToohardRollerConstants.COLLECTION_VOLTAGE_COMPENSATION_SATURATION;
    }
}
