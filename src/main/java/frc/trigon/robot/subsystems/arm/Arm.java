package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private final static Arm INSTANCE = new Arm();
    private final CANSparkMax masterAngleMotor = ArmConstants.MASTER_ANGLE_MOTOR;
    private final CANSparkMax followerAngleMotor = ArmConstants.FOLLOWER_ANGLE_MOTOR;
    private final CANSparkMax masterElevatorMotor = ArmConstants.MASTER_ELEVATOR_MOTOR;
    private final CANSparkMax followerElevatorMotor = ArmConstants.FOLLOWER_ELEVATOR_MOTOR;

    public static Arm getInstance() {
        return INSTANCE;
    }

    private Arm() {

    }
}

