package frc.trigon.robot.subsystems.arm.simulationarm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.arm.ArmIO;
import frc.trigon.robot.subsystems.arm.ArmInputsAutoLogged;
import frc.trigon.robot.subsystems.arm.kablamaArm.KablamaArmConstants;

public class SimulationArmIO extends ArmIO {
    private double
            angleVoltage = 0,
            elevatorVoltage = 0;
    private final SingleJointedArmSim
            masterAngleMotor = SimulationArmIOConstants.MASTER_ANGLE_MOTOR,
            followerAngleMotor = SimulationArmIOConstants.FOLLOWER_ANGLE_MOTOR;
    private final ElevatorSim
            masterElevatorMotor = SimulationArmIOConstants.MASTER_ELEVATOR_MOTOR,
            followerElevatorMotor = SimulationArmIOConstants.FOLLOWER_ELEVATOR_MOTOR;

    @Override
    protected void updateInputs(ArmInputsAutoLogged inputs) {
        masterAngleMotor.update(RobotConstants.PERIODIC_TIME_SECONDS);
        followerAngleMotor.update(RobotConstants.PERIODIC_TIME_SECONDS);
        masterElevatorMotor.update(RobotConstants.PERIODIC_TIME_SECONDS);
        followerElevatorMotor.update(RobotConstants.PERIODIC_TIME_SECONDS);

        inputs.angleMotorVoltage = angleVoltage;
        inputs.angleMotorCurrent = masterAngleMotor.getCurrentDrawAmps();
        inputs.angleVelocityDegreesPerSecond = getAngleVelocityDegreesPerSecond();

        inputs.elevatorMotorVoltage = elevatorVoltage;
        inputs.elevatorMotorCurrent = masterElevatorMotor.getCurrentDrawAmps();
        inputs.elevatorPositionRevolution = masterElevatorMotor.getPositionMeters();
        inputs.elevatorVelocityRevolutionsPerSecond = masterElevatorMotor.getVelocityMetersPerSecond();
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
        setAngleVoltage(calculateAngleOutput(targetState));
    }

    @Override
    protected void setTargetElevatorState(TrapezoidProfile.State targetState) {
        setElevatorVoltage(calculateElevatorOutput(targetState));
    }

    private void setAngleVoltage(double voltage) {
        this.angleVoltage = voltage;
        masterAngleMotor.setInputVoltage(voltage);
        followerAngleMotor.setInputVoltage(voltage);
    }

    private void setElevatorVoltage(double voltage) {
        this.elevatorVoltage = voltage;
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

    private double getAngleVelocityDegreesPerSecond(){
        return Rotation2d.fromRadians(masterAngleMotor.getAngleRads()).getDegrees();
    }
}
