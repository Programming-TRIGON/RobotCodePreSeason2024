package frc.trigon.robot.subsystems.arm.simulationarm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.arm.ArmIO;
import frc.trigon.robot.subsystems.arm.ArmInputsAutoLogged;
import org.littletonrobotics.junction.LogTable;

public class SimulationArmIO extends ArmIO {
    private final SingleJointedArmSim angleMotor = SimulationArmIOConstants.ANGLE_MOTOR;
    private final ElevatorSim elevatorMotor = SimulationArmIOConstants.ELEVATOR_MOTOR;
    private final Mechanism2d mechanism2d = SimulationArmIOConstants.MECHANISM2D;
    private double
            angleVoltage = 0,
            elevatorVoltage = 0;

    @Override
    protected void updateInputs(ArmInputsAutoLogged inputs) {
        angleMotor.update(RobotConstants.PERIODIC_TIME_SECONDS);
        elevatorMotor.update(RobotConstants.PERIODIC_TIME_SECONDS);

        inputs.angleMotorVoltage = angleVoltage;
        inputs.angleMotorCurrent = angleMotor.getCurrentDrawAmps();
        inputs.angleVelocityDegreesPerSecond = getAngleVelocityDegreesPerSecond();
        inputs.anglePositionDegrees = Units.radiansToDegrees(angleMotor.getAngleRads());

        inputs.elevatorMotorVoltage = elevatorVoltage;
        inputs.elevatorMotorCurrent = elevatorMotor.getCurrentDrawAmps();
        inputs.elevatorPositionRevolution = getElevatorPositionRevolutions();
        inputs.elevatorVelocityRevolutionsPerSecond = elevatorMotor.getVelocityMetersPerSecond() / SimulationArmIOConstants.METERS_PER_REVOLUTION;

        inputs.mechanism2d = mechanism2d;
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

    @Override
    protected void startAdvantageKitLogging(LogTable logTable) {
        mechanism2d.akitLog(logTable);
        SmartDashboard.putData("Mech2d", mechanism2d);
    }

    private void setAngleVoltage(double voltage) {
        this.angleVoltage = voltage;
        angleMotor.setInputVoltage(voltage);
    }

    private void setElevatorVoltage(double voltage) {
        this.elevatorVoltage = voltage;
        elevatorMotor.setInputVoltage(voltage);
    }

    private double calculateAngleOutput(TrapezoidProfile.State targetState) {
        double pidOutput = SimulationArmIOConstants.ANGLE_PID_CONTROLLER.calculate(
                angleMotor.getAngleRads(),
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
                getElevatorPositionRevolutions(),
                targetState.position
        );
        double feedforward = SimulationArmIOConstants.ELEVATOR_FEEDFORWARD.calculate(targetState.velocity);
        return pidOutput + feedforward;
    }

    private double getAngleVelocityDegreesPerSecond() {
        return Units.radiansToDegrees(angleMotor.getAngleRads());
    }

    private double getElevatorPositionRevolutions() {
        double offsettedPosition = elevatorMotor.getPositionMeters() - SimulationArmIOConstants.RETRACTED_ARM_LENGTH_METERS;
        return offsettedPosition / SimulationArmIOConstants.METERS_PER_REVOLUTION;
    }
}
