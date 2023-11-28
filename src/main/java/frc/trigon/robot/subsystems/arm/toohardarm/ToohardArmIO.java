package frc.trigon.robot.subsystems.arm.toohardarm;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.subsystems.arm.ArmIO;
import frc.trigon.robot.subsystems.arm.ArmInputsAutoLogged;
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
        inputs.angleEncoderPosition = getAngleEncoderPositionRotations();
        inputs.angleEncoderVelocity = getAngleEncoderVelocityRotationsPerSecond();
    }

    @Override
    protected void setTargetAngle(TrapezoidProfile.State targetState) {
        setAngleMotorsPower(targetState);
    }

    @Override
    protected void setTargetElevatorPosition(TrapezoidProfile.State targetState) {
        setElevatorMotorsPower(targetState);
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

    private double setElevatorPowerFromProfile(TrapezoidProfile.State targetState) {
        double pidOutput = ToohardArmConstants.ELEVATOR_PID_CONTROLLER.calculate(
                getElevatorPositionRevolutions(),
                targetState.position
        );
        double feedforward = ToohardArmConstants.ELEVATOR_FEEDFORWARD.calculate(targetState.velocity);
        return pidOutput + feedforward;
    }

    private double setAnglePowerFromProfile(TrapezoidProfile.State targetState) {
        double pidOutput = ToohardArmConstants.ANGLE_PID_CONTROLLER.calculate(
                getAngleEncoderPositionRotations(),
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
        return Conversions.perHundredMsToPerSecond(Conversions.magTicksToRevolutions(elevatorEncoder.getSelectedSensorVelocity()));
    }

    private double getAngleEncoderPositionRotations() {
        return ToohardArmConstants.ANGLE_ENCODER_POSITION_SIGNAL.refresh().getValue();
    }

    private double getAngleEncoderVelocityRotationsPerSecond() {
        return Conversions.perHundredMsToPerSecond(Conversions.revolutionsToDegrees(ToohardArmConstants.ANGLE_ENCODER_VELOCITY_SIGNAL.refresh().getValue()));
    }

    private void setAngleMotorsPower(TrapezoidProfile.State targetState)    {
        masterAngleMotor.set(setAnglePowerFromProfile(targetState));
        followerAngleMotor.set(setAnglePowerFromProfile(targetState));
    }

    private void setElevatorMotorsPower(TrapezoidProfile.State targetState)    {
        masterElevatorMotor.set(setElevatorPowerFromProfile(targetState));
        followerElevatorMotor.set(setElevatorPowerFromProfile(targetState));
    }
}
