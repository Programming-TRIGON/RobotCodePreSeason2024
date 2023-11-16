package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private static final CANSparkMax
            masterAngleMotor = ArmConstants.MASTER_ANGLE_MOTOR,
            followerAngleMotor = ArmConstants.FOLLOWER_ANGLE_MOTOR,
            masterElevatorMotor = ArmConstants.MASTER_ELEVATOR_MOTOR,
            followerElevatorMotor = ArmConstants.FOLLOWER_ELEVATOR_MOTOR;
    private static final TalonSRX elevatorEncoder = ArmConstants.ELEVATOR_ENCODER;
    private static final CANcoder angleEncoder = ArmConstants.ANGLE_ENCODER;

    private static final ArmFeedforward angleMotorFeedforward = ArmConstants.ANGLE_MOTOR_FEEDFORWARD;
    private static final ElevatorFeedforward elevatorMotorFeedforward = ArmConstants.ELEVATOR_MOTOR_FEEDFORWARD;

    private static final TrapezoidProfile.Constraints
            angleConstrains = ArmConstants.ANGLE_CONSTRAINS,
            elevatorConstrains = ArmConstants.ELEVATOR_CONSTRAINS;

    private final static ArmSubsystem INSTANCE = new ArmSubsystem();
    @SuppressWarnings("WeakerAccess")
    public static ArmSubsystem getInstance() {
        return INSTANCE;
    }

    private ArmSubsystem() {
    }
}

