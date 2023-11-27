package frc.trigon.robot.subsystems.arm.toohardarm;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
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
    private final double angleEncoderPositionSignal = ToohardArmConstants.ANGLE_ENCODER_POSITION_SIGNAL.getValue();
    private final double angleEncoderVelocitySignal = ToohardArmConstants.ANGLE_ENCODER_VELOCITY_SIGNAL.getValue();

    @Override
    protected void updateInputs(ArmInputsAutoLogged inputs) {
        inputs.angleMotorsVoltage = masterAngleMotor.getBusVoltage();
        inputs.elevatorMotorsVoltage = masterElevatorMotor.getBusVoltage();

        inputs.elevatorPositionRevolutions = Conversions.magTicksToRevolutions(elevatorEncoder.getSelectedSensorPosition());
        inputs.elevatorVelocityRevolutionsPerSecond = Conversions.perHundredMsToPerSecond(Conversions.magTicksToRevolutions(elevatorEncoder.getSelectedSensorVelocity()));

        inputs.angleEncoderPositionSignal = angleEncoderPositionSignal;
        inputs.angleEncoderVelocitySignal = angleEncoderVelocitySignal;
    }

    @Override
    protected void setTargetAngle(Rotation2d targetAngle) {
        double pidOutput = ToohardArmConstants.ANGLE_PID_CONTROLLER.calculate(
                angleEncoderPositionSignal,
                targetAngle.getDegrees()
        );
        double feedforward = ToohardArmConstants.ANGLE_FEEDFORWARD.calculate(
                targetAngle.getRadians(),
                Units.degreesToRadians(angleEncoderVelocitySignal)
        );
        masterAngleMotor.set(pidOutput + feedforward);
        followerAngleMotor.set(pidOutput + feedforward);
    }

    @Override
    protected void setTargetElevatorPosition(double targetPosition) {
        double pidOutput = ToohardArmConstants.ELEVATOR_PID_CONTROLLER.calculate(
                elevatorEncoder.getSelectedSensorPosition(),
                targetPosition
        );
        double feedforward = ToohardArmConstants.ELEVATOR_FEEDFORWARD.calculate(
                Units.degreesToRadians(elevatorEncoder.getSelectedSensorPosition()),
                Units.degreesToRadians(elevatorEncoder.getSelectedSensorVelocity())
        );
        masterElevatorMotor.set(pidOutput + feedforward);
        followerElevatorMotor.set(pidOutput + feedforward);
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
}
