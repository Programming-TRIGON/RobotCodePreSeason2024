package frc.trigon.robot.subsystems.roller.simulationroller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.roller.RollerIO;
import frc.trigon.robot.subsystems.roller.RollerInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;
import org.littletonrobotics.junction.Logger;

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
        inputs.forwardLimitSwitchPressed = isForwardLimitSwitchPressed();
        inputs.backwardLimitSwitchPressed = isBackwardLimitSwitchPressed();

        Logger.recordOutput("Roller/RollerAngle", Units.radiansToDegrees(angleSimulation.getAngleRads()));
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
        angleSimulation.setInputVoltage(angleMotorVoltage);
    }

    private void setCollectorMotorVoltage(double voltage) {
        collectorMotorVoltage = MathUtil.clamp(
                voltage,
                -SimulationRollerConstants.VOLTAGE_COMPENSATION_SATURATION,
                SimulationRollerConstants.VOLTAGE_COMPENSATION_SATURATION
        );
        collectorSimulation.setInputVoltage(collectorMotorVoltage);
    }

    private boolean isForwardLimitSwitchPressed() {
        return !angleSimulation.hasHitUpperLimit();
    }

    private boolean isBackwardLimitSwitchPressed() {
        return !angleSimulation.hasHitLowerLimit();
    }
}
