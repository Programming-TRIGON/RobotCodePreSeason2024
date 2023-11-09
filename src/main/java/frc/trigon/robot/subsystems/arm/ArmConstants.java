package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;

public class ArmConstants {
    private static final int
        MASTER_ANGLE_MOTOR_ID = 0,
        FOLLOWER_ANGLE_MOTOR_ID = 1,
        MASTER_ELEVATOR_MOTOR_ID = 2,
        FOLLOWER_ELEVATOR_MOTOR_ID = 3,
        MAG_ENCODER_ID = 4,
        ANGLE_CANCODER_ID = 5;

    private static final double
            MASTER_ANGLE_OFFSET_VALUE = 0.0,
            FOLLOWER_ANGLE_OFFSET_VALUE = 0.0,
            MASTER_ELEVATOR_OFFSET_VALUE = 0.0,
            FOLLOWER_ELEVATOR_OFFSET_VALUE = 0.0,
            MAG_ENCODER_OFFSET = 0.0,
            ANGLE_CANCODER_OFFSET = 0.0;

    private static final boolean
            MASTER_ANGLE_INVERTED_VALUE = true,
            FOLLOWER_ANGLE_INVERTED_VALUE = true,
            MASTER_ELEVATOR_INVERTED_VALUE = false,
            FOLLOWER_ELEVATOR_INVERTED_VALUE = true,
            MAG_ENCODER_INVERTED_VALUE = true;
    private static final SensorDirectionValue CANCODER_INVERTED_VALUE = SensorDirectionValue.Clockwise_Positive;

    private static final double
        P = 1.32,
        I = 3.1,
        D = 0;
    PIDController PID_CONTROLLER = new PIDController(P,I,D);

    private static final CANSparkMax.IdleMode
        MASTER_ANGLE_NEUTRAL_MODE_VALUE  = CANSparkMax.IdleMode.kBrake,
        FOLLOWER_ANGLE_NEUTRAL_MODE_VALUE = CANSparkMax.IdleMode.kBrake,
        MASTER_ELEVATOR_NEUTRAL_MODE_VALUE = CANSparkMax.IdleMode.kBrake,
        FOLLOWER_ELEVATOR_NEUTRAL_MODE_VALUE = CANSparkMax.IdleMode.kBrake;
    private static final AbsoluteSensorRangeValue ANGLE_ENCODER_SENSOR_RANGE = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

    private static final CANSparkMax
        MASTER_ANGLE_MOTOR = new CANSparkMax(MASTER_ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
        FOLLOWER_ANGLE_MOTOR = new CANSparkMax(FOLLOWER_ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
        MASTER_ELEVATOR_MOTOR = new CANSparkMax(MASTER_ELEVATOR_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
        FOLLOWER_ELEVATOR_MOTOR = new CANSparkMax(FOLLOWER_ELEVATOR_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private static final TalonSRX MAG_ENCODER = new TalonSRX(MAG_ENCODER_ID);
    private static final CANcoder CANCODER = new CANcoder(ANGLE_CANCODER_ID);

    private static void config_elevator_motors(){
        MASTER_ELEVATOR_MOTOR.setInverted(MASTER_ELEVATOR_INVERTED_VALUE);
        MASTER_ELEVATOR_MOTOR.setIdleMode(MASTER_ANGLE_NEUTRAL_MODE_VALUE);

        FOLLOWER_ELEVATOR_MOTOR.setInverted(FOLLOWER_ANGLE_INVERTED_VALUE);
        FOLLOWER_ELEVATOR_MOTOR.setIdleMode(FOLLOWER_ANGLE_NEUTRAL_MODE_VALUE);

        MAG_ENCODER.setSelectedSensorPosition(MAG_ENCODER.getSelectedSensorPosition() - MAG_ENCODER_OFFSET);
        MAG_ENCODER.setSensorPhase(MAG_ENCODER_INVERTED_VALUE);

    }


    private static void config_angle_motors(){
        MASTER_ANGLE_MOTOR.setInverted(MASTER_ELEVATOR_INVERTED_VALUE);
        MASTER_ANGLE_MOTOR.setIdleMode(MASTER_ANGLE_NEUTRAL_MODE_VALUE);

        FOLLOWER_ANGLE_MOTOR.setInverted(FOLLOWER_ANGLE_INVERTED_VALUE);
        FOLLOWER_ANGLE_MOTOR.setIdleMode(FOLLOWER_ANGLE_NEUTRAL_MODE_VALUE);

        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = ANGLE_CANCODER_OFFSET;
        config.MagnetSensor.AbsoluteSensorRange = ANGLE_ENCODER_SENSOR_RANGE;
        config.MagnetSensor.SensorDirection = CANCODER_INVERTED_VALUE;
        CANCODER.getConfigurator().apply(config);
    }

    static {
        config_angle_motors();
        config_elevator_motors();
    }
    public enum ArmStates {
        FIRST_STATE(43, 1),
        SECOND_STATE(50, 2),
        THIRD_STATE(100, 3);
        final double rotation2d;
        final int elevatorPosition;

        ArmStates(double rotation2d, int elevatorPosition){
            this.elevatorPosition = elevatorPosition;
            this.rotation2d = rotation2d;
        }

    }

}
