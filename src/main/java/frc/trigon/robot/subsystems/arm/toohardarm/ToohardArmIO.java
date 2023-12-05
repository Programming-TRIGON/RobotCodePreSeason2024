package frc.trigon.robot.subsystems.arm.toohardarm;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.subsystems.arm.ArmIO;
import frc.trigon.robot.subsystems.arm.ArmInputsAutoLogged;
import frc.trigon.robot.subsystems.arm.simulationarm.SimulationArmConstants;
import frc.trigon.robot.utilities.Conversions;

public class ToohardArmIO extends ArmIO {
    private final CANSparkMax
            masterAngleMotor = ToohardArmConstants.MASTER_ANGLE_MOTOR,
            followerAngleMotor = ToohardArmConstants.FOLLOWER_ANGLE_MOTOR,
            masterElevatorMotor = ToohardArmConstants.MASTER_ELEVATOR_MOTOR,
            followerElevatorMotor = ToohardArmConstants.FOLLOWER_ELEVATOR_MOTOR;
    private final TalonSRX elevatorEncoder = ToohardArmConstants.ELEVATOR_ENCODER;

    @Override
    protected void updateInputs(ArmInputsAutoLogged inputs) {
        inputs.elevatorMotorVoltage = masterElevatorMotor.getBusVoltage();
        inputs.elevatorPositionRevolutions = getElevatorPositionRevolutions();
        inputs.elevatorVelocityRevolutionsPerSecond = getElevatorVelocityRevolutionsPerSecond();

        inputs.angleMotorVoltage = masterAngleMotor.getBusVoltage();
        inputs.anglePositionDegrees = getAnglePositionDegrees();
        inputs.angleVelocityDegreesPerSecond = getAngleVelocityDegreesPerSecond();
    }

    @Override
    protected void setTargetAngleState(TrapezoidProfile.State targetState) {
        setAngleMotorsPower(calculateAnglePowerFromState(targetState));
    }

    @Override
    protected void setTargetElevatorState(TrapezoidProfile.State targetState) {
        setElevatorMotorsPower(calculateElevatorPowerFromState(targetState));
    }

    @Override
    protected void stopAngleMotors() {
        masterAngleMotor.stopMotor();
        followerAngleMotor.stopMotor();
    }

    @Override
    protected void stopElevatorMotors() {
        masterElevatorMotor.stopMotor();
        followerElevatorMotor.stopMotor();
    }

    private double calculateElevatorPowerFromState(TrapezoidProfile.State targetState) {
        double pidOutput = ToohardArmConstants.ELEVATOR_PID_CONTROLLER.calculate(
                getElevatorPositionRevolutions(),
                targetState.position
        );
        double feedforward = ToohardArmConstants.ELEVATOR_FEEDFORWARD.calculate(targetState.velocity);
        return pidOutput + feedforward;
    }

    private double calculateAnglePowerFromState(TrapezoidProfile.State targetState) {
        double pidOutput = ToohardArmConstants.ANGLE_PID_CONTROLLER.calculate(
                getAnglePositionDegrees(),
                targetState.position
        );
        double feedforward = ToohardArmConstants.ANGLE_FEEDFORWARD.calculate(
                Units.degreesToRadians(targetState.position),
                targetState.velocity
        );
        return pidOutput + feedforward;
    }

    private double getElevatorPositionRevolutions() {
        return Conversions.magTicksToRevolutions(elevatorEncoder.getSelectedSensorPosition());
    }

    private double getElevatorVelocityRevolutionsPerSecond() {
        double elevatorVelocityRevolutions = Conversions.magTicksToRevolutions(elevatorEncoder.getSelectedSensorVelocity());
        return Conversions.perHundredMsToPerSecond(elevatorVelocityRevolutions);
    }

    private double getAnglePositionDegrees() {
        return Conversions.revolutionsToDegrees(ToohardArmConstants.ANGLE_ENCODER_POSITION_SIGNAL.refresh().getValue());
    }

    private double getAngleVelocityDegreesPerSecond() {
        return Conversions.revolutionsToDegrees(ToohardArmConstants.ANGLE_ENCODER_VELOCITY_SIGNAL.refresh().getValue());
    }

    private void setAngleMotorsPower(double power) {
        masterAngleMotor.set(powerToVoltage(power));
        followerAngleMotor.set(powerToVoltage(power));
    }

    private void setElevatorMotorsPower(double power) {
        masterElevatorMotor.set(powerToVoltage(power));
        followerElevatorMotor.set(powerToVoltage(power));
    }

    private double powerToVoltage(double power) {
        return Conversions.compensatedPowerToVoltage(power, ToohardArmConstants.VOLTAGE_COMPENSATION_SATURATION);
    }
}
