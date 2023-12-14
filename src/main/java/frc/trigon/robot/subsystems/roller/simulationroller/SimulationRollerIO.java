package frc.trigon.robot.subsystems.roller.simulationroller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.roller.RollerIO;
import frc.trigon.robot.subsystems.roller.RollerInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;

public class SimulationRollerIO extends RollerIO {
    private final SingleJointedArmSim angleMotor = SimulationRollerConstants.ANGLE_MOTOR;
    private final DCMotorSim collectorMotor = SimulationRollerConstants.COLLECTOR_MOTOR;

    private double angleMotorVoltage, collectorMotorVoltage;

    @Override
    protected void updateInputs(RollerInputsAutoLogged inputs) {
        angleMotor.update(RobotConstants.PERIODIC_TIME_SECONDS);
        collectorMotor.update(RobotConstants.PERIODIC_TIME_SECONDS);

        inputs.angleMotorVoltage = angleMotorVoltage;
        inputs.angleMotorCurrent = angleMotor.getCurrentDrawAmps();
        inputs.collectionMotorVoltage = collectorMotorVoltage;
        inputs.collectionMotorCurrent = collectorMotor.getCurrentDrawAmps();
        inputs.forwardLimitSwitchPressed = angleMotor.hasHitUpperLimit();
        inputs.backwardLimitSwitchPressed = angleMotor.hasHitLowerLimit();
        
        if (DriverStation.isDisabled()) {
            stopAngleMotor();
            stopCollectionMotor();
        }
    }

    @Override
    protected void setAngleMotorPower(double power) {
        setAngleMotorVoltage(powerToVoltage(power));
    }

    @Override
    protected void setCollectionMotorPower(double power) {
        setCollectorMotorVoltage(powerToVoltage(power));
    }

    @Override
    protected void stopAngleMotor() {
        setAngleMotorVoltage(0);
    }

    @Override
    protected void stopCollectionMotor() {
        setCollectorMotorVoltage(0);
    }

    private double powerToVoltage(double power) {
        return Conversions.compensatedPowerToVoltage(power, SimulationRollerConstants.VOLTAGE_COMPENSATION_SATURATION);
    }

    private void setAngleMotorVoltage(double voltage) {
        angleMotorVoltage = MathUtil.clamp(
                voltage,
                -SimulationRollerConstants.VOLTAGE_COMPENSATION_SATURATION,
                SimulationRollerConstants.VOLTAGE_COMPENSATION_SATURATION
        );
        angleMotor.setInputVoltage(angleMotorVoltage);
    }

    private void setCollectorMotorVoltage(double voltage) {
        collectorMotorVoltage = MathUtil.clamp(
                voltage,
                -SimulationRollerConstants.VOLTAGE_COMPENSATION_SATURATION,
                SimulationRollerConstants.VOLTAGE_COMPENSATION_SATURATION
        );
        collectorMotor.setInputVoltage(collectorMotorVoltage);
    }
}
