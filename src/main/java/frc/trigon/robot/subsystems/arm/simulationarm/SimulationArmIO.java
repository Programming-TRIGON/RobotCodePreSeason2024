package frc.trigon.robot.subsystems.arm.simulationarm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.arm.ArmIO;
import frc.trigon.robot.subsystems.arm.ArmInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;

public class SimulationArmIO extends ArmIO {
    private final SingleJointedArmSim angleSimulation = SimulationArmConstants.ANGLE_MOTOR;
    private final ElevatorSim elevatorSimulation = SimulationArmConstants.ELEVATOR_MOTOR;

    private double angleMotorVoltage, elevatorMotorVoltage;

    @Override
    protected void updateInputs(ArmInputsAutoLogged inputs) {
        angleSimulation.update(RobotConstants.PERIODIC_TIME_SECONDS);
        elevatorSimulation.update(RobotConstants.PERIODIC_TIME_SECONDS);

        inputs.angleMotorVoltage = angleMotorVoltage;
        inputs.anglePositionDegrees = getAnglePositionDegrees();
        inputs.angleVelocityDegreesPerSecond = getAngleVelocityDegreesPerSecond();
        inputs.elevatorMotorVoltage = elevatorMotorVoltage;
        inputs.elevatorPositionRevolutions = getElevatorPositionRevolutions();
        inputs.elevatorVelocityRevolutionsPerSecond = getElevatorVelocityRevolutionsPerSecond();
    }

    @Override
    protected void setTargetAngleState(TrapezoidProfile.State targetState) {
        setAngleMotorsVoltage(calculateAngleVoltageFromState(targetState));
    }

    @Override
    protected void setTargetElevatorState(TrapezoidProfile.State targetState) {
        setElevatorMotorsVoltage(calculateElevatorVoltageFromState(targetState));
    }

    @Override
    protected void stopAngleMotors() {
        setAngleMotorsVoltage(0);
    }

    @Override
    protected void stopElevatorMotors() {
        setElevatorMotorsVoltage(0);
    }

    private double calculateElevatorVoltageFromState(TrapezoidProfile.State targetState) {
        double pidOutput = SimulationArmConstants.ELEVATOR_PID_CONTROLLER.calculate(
                getElevatorPositionRevolutions(),
                targetState.position
        );
        double feedforward = SimulationArmConstants.ELEVATOR_FEEDFORWARD.calculate(targetState.velocity);
        return Conversions.compensatedPowerToVoltage(pidOutput + feedforward, SimulationArmConstants.VOLTAGE_COMPENSATION_SATURATION);
    }

    private double calculateAngleVoltageFromState(TrapezoidProfile.State targetState) {
        double pidOutput = SimulationArmConstants.ANGLE_PID_CONTROLLER.calculate(
                getAnglePositionDegrees(),
                targetState.position
        );
        double feedforward = SimulationArmConstants.ANGLE_FEEDFORWARD.calculate(
                Units.degreesToRadians(targetState.position),
                targetState.velocity
        );
        return Conversions.compensatedPowerToVoltage(pidOutput + feedforward, SimulationArmConstants.VOLTAGE_COMPENSATION_SATURATION);
    }

    private double getElevatorPositionRevolutions() {
        return Conversions.distanceToRevolutions(elevatorSimulation.getPositionMeters(), SimulationArmConstants.ELEVATOR_METERS_PER_REVOLUTION);
    }

    private double getElevatorVelocityRevolutionsPerSecond() {
        return Conversions.distanceToRevolutions(elevatorSimulation.getVelocityMetersPerSecond(), SimulationArmConstants.ELEVATOR_METERS_PER_REVOLUTION);
    }

    private double getAnglePositionDegrees() {
        return Units.radiansToDegrees(angleSimulation.getAngleRads());
    }

    private double getAngleVelocityDegreesPerSecond() {
        return Units.radiansToDegrees(angleSimulation.getVelocityRadPerSec());
    }

    private void setAngleMotorsVoltage(double voltage) {
        angleMotorVoltage = MathUtil.clamp(
                voltage,
                -SimulationArmConstants.VOLTAGE_COMPENSATION_SATURATION,
                SimulationArmConstants.VOLTAGE_COMPENSATION_SATURATION
        );
        angleSimulation.setInputVoltage(voltage);
    }

    private void setElevatorMotorsVoltage(double voltage) {
        elevatorMotorVoltage = MathUtil.clamp(
                voltage,
                -SimulationArmConstants.VOLTAGE_COMPENSATION_SATURATION,
                SimulationArmConstants.VOLTAGE_COMPENSATION_SATURATION
        );
        elevatorSimulation.setInputVoltage(elevatorMotorVoltage);
    }
}