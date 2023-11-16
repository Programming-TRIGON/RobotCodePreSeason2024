package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private final static Arm INSTANCE = new Arm();

    public static Arm getInstance() {return INSTANCE;}

    private final CANSparkMax
            masterAngleMotor = frc.trigon.robot.subsystems.arm.ArmConstants.MASTER_ANGLE_MOTOR,
            followerAngleMotor = frc.trigon.robot.subsystems.arm.ArmConstants.FOLLOWER_ANGLE_MOTOR,
            masterElevatorMotor = frc.trigon.robot.subsystems.arm.ArmConstants.MASTER_ELEVATOR_MOTOR,
            followerElevatorMotor = frc.trigon.robot.subsystems.arm.ArmConstants.FOLLOWER_ELEVATOR_MOTOR;
    private final TalonSRX elevatorEncoder = frc.trigon.robot.subsystems.arm.ArmConstants.ELEVATOR_ENCODER;
    private final CANcoder angleEncoder = frc.trigon.robot.subsystems.arm.ArmConstants.ANGLE_ENCODER;

    private final ArmFeedforward angleMotorFeedforward = frc.trigon.robot.subsystems.arm.ArmConstants.ANGLE_MOTOR_FEEDFORWARD;
    private final ElevatorFeedforward elevatorMotorFeedforward = frc.trigon.robot.subsystems.arm.ArmConstants.ELEVATOR_MOTOR_FEEDFORWARD;

    private Arm() {
    }
}
