package frc.trigon.robot.subsystems.arm.simulationarm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.trigon.robot.subsystems.arm.ArmIO;
import frc.trigon.robot.subsystems.arm.ArmInputsAutoLogged;
import frc.trigon.robot.subsystems.arm.kablamaArm.KablamaArmConstants;

public class SimulationArmIO extends ArmIO {
    private final SingleJointedArmSim
            masterAngleMotor = SimulationArmIOConstants.MASTER_ANGLE_MOTOR,
            followerAngleMotor = SimulationArmIOConstants.FOLLOWER_ANGLE_MOTOR;
    private final ElevatorSim
            masterElevatorMotor = SimulationArmIOConstants.MASTER_ELEVATOR_MOTOR,
            followerElevatorMotor = SimulationArmIOConstants.FOLLOWER_ELEVATOR_MOTOR;

    @Override
    protected void updateInputs(ArmInputsAutoLogged inputs) {
    }

    @Override
    protected void stopAngleMotors() {
        setAngleVoltage(0);
    }

    @Override
    protected void stopElevatorMotors() {
        setElevatorVoltage(0);
    }

    @Override
    protected void setTargetAngleState(TrapezoidProfile.State targetState) {

    }

    @Override
    protected void setTargetElevatorState(TrapezoidProfile.State targetState) {
    }

    private void setAngleVoltage(double voltage) {
        masterAngleMotor.setInputVoltage(voltage);
        followerAngleMotor.setInputVoltage(voltage);
    }

    private void setElevatorVoltage(double voltage) {
        masterElevatorMotor.setInputVoltage(voltage);
        followerElevatorMotor.setInputVoltage(voltage);
    }

    private double calculateAngleOutput(TrapezoidProfile.State targetState) {
        double pidOutput = SimulationArmIOConstants.ANGLE_PID_CONTROLLER.calculate(
                masterAngleMotor.getAngleRads(),
                targetState.position
        );
        double feedforward = SimulationArmIOConstants.ANGLE_FEEDFORWARD.calculate(
                Units.degreesToRadians(targetState.position),
                Units.degreesToRadians(targetState.velocity)
        );
        return pidOutput + feedforward;
    }

    private double calculateElevatorOutput(TrapezoidProfile.State targetState) {
        double pidOutput = SimulationArmIOConstants.ELEVATOR_PID_CONTROLLER.calculate(
                masterElevatorMotor.getPositionMeters(),
                targetState.position
        );
        double feedforward = SimulationArmIOConstants.ELEVATOR_FEEDFORWARD.calculate(targetState.velocity);
        return pidOutput + feedforward;
    }
}
