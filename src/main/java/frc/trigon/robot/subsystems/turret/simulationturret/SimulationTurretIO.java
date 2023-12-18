package frc.trigon.robot.subsystems.turret.simulationturret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.turret.TurretIO;
import frc.trigon.robot.subsystems.turret.TurretInputsAutoLogged;

public class SimulationTurretIO extends TurretIO {
    private final SingleJointedArmSim motor = SimulationTurretConstants.MOTOR;

    private double motorVoltage;

    @Override
    protected void updateInputs(TurretInputsAutoLogged inputs) {
        motor.update(RobotConstants.PERIODIC_TIME_SECONDS);

        inputs.motorVoltage = motorVoltage;
        inputs.motorAngleDegrees = Units.radiansToDegrees(motor.getAngleRads());
        inputs.motorVelocityDegreesPerSecond = Units.radiansToDegrees(motor.getVelocityRadPerSec());
    }

    @Override
    protected void setTargetAngleState(TrapezoidProfile.State targetState) {
        setMotorVoltage(calculateVoltageFromState(targetState));
    }

    @Override
    protected void stopMotor() {
        setMotorVoltage(0);
    }

    private double calculateVoltageFromState(TrapezoidProfile.State targetState) {
        double pidOutput = SimulationTurretConstants.PID_CONTROLLER.calculate(
                Units.radiansToDegrees(motor.getAngleRads()),
                targetState.position
        );
        double feedforward = SimulationTurretConstants.FEEDFORWARD.calculate(
                Units.degreesToRadians(targetState.position),
                targetState.velocity
        );
        return pidOutput + feedforward;
    }

    private void setMotorVoltage(double voltage) {
        motorVoltage = MathUtil.clamp(
                voltage,
                -SimulationTurretConstants.VOLTAGE_COMPENSATION_SATURATION,
                SimulationTurretConstants.VOLTAGE_COMPENSATION_SATURATION
        );
        motor.setInputVoltage(motorVoltage);
    }
}
