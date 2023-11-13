package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ArmConstants {
    private static final int
            MASTER_ANGLE_MOTOR_ID = 0,
            FOLLOWER_ANGLE_MOTOR_ID = 1,
            MASTER_ELEVATOR_MOTOR_ID = 2,
            FOLLOWER_ELEVATOR_MOTOR_ID = 3,
            ELEVATOR_MAG_ENCODER_ID = 4,
            ANGLE_CANCODER_ID = 5;

    private static final double
            ELEVATOR_MAG_ENCODER_OFFSET = 0.0,
            ANGLE_CANCODER_OFFSET = 0.0;

    private static final boolean
            MASTER_ANGLE_INVERTED_VALUE = false,
            FOLLOWER_ANGLE_INVERTED_VALUE = false,
            MASTER_ELEVATOR_INVERTED_VALUE = false,
            FOLLOWER_ELEVATOR_INVERTED_VALUE = true,
            ELEVATOR_MAG_ENCODER_PHASE_VALUE = true;
    private static final SensorDirectionValue ANGLE_CANCODER_SENSOR_DIRECTION = SensorDirectionValue.Clockwise_Positive;

    private static final double
            P = 1.32,
            I = 3.1,
            D = 0;
    private static final PIDController PID_CONTROLLER = new PIDController(P, I, D);

    private static final double
        MAX_ANGLE_VELOCITY = 600,
        MAX_ANGLE_ACCELERATION = 500,
        MAX_ELEVATOR_VELOCITY = 900,
        MAX_ELEVATOR_ACCELERATION = 550;
    static final TrapezoidProfile.Constraints ANGLE_CONSTRAINS = new TrapezoidProfile.Constraints(MAX_ANGLE_VELOCITY, MAX_ANGLE_ACCELERATION);
    static final TrapezoidProfile.Constraints ELEVATOR_CONSTRAINS = new TrapezoidProfile.Constraints(MAX_ELEVATOR_VELOCITY, MAX_ELEVATOR_ACCELERATION);


    private static final CANSparkMax.IdleMode
            ANGLE_NEUTRAL_MODE_VALUE = CANSparkMax.IdleMode.kBrake,
            ELEVATOR_NEUTRAL_MODE_VALUE = CANSparkMax.IdleMode.kBrake;
    private static final AbsoluteSensorRangeValue ANGLE_ENCODER_SENSOR_RANGE = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

    static final CANSparkMax
            MASTER_ANGLE_MOTOR = new CANSparkMax(MASTER_ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
            FOLLOWER_ANGLE_MOTOR = new CANSparkMax(FOLLOWER_ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
            MASTER_ELEVATOR_MOTOR = new CANSparkMax(MASTER_ELEVATOR_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
            FOLLOWER_ELEVATOR_MOTOR = new CANSparkMax(FOLLOWER_ELEVATOR_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    static final TalonSRX ELEVATOR_MAG_ENCODER = new TalonSRX(ELEVATOR_MAG_ENCODER_ID);
    static final CANcoder ANGLE_CANCODER = new CANcoder(ANGLE_CANCODER_ID);

    static {
        motors_angle_config();
        motors_elevator_config();
        configureElevatorEncoder();
    }

    private static void motors_elevator_config() {
        MASTER_ELEVATOR_MOTOR.restoreFactoryDefaults();
        FOLLOWER_ELEVATOR_MOTOR.restoreFactoryDefaults();

        MASTER_ELEVATOR_MOTOR.setInverted(MASTER_ELEVATOR_INVERTED_VALUE);
        FOLLOWER_ELEVATOR_MOTOR.setInverted(FOLLOWER_ELEVATOR_INVERTED_VALUE);

        MASTER_ELEVATOR_MOTOR.setIdleMode(ANGLE_NEUTRAL_MODE_VALUE);
        FOLLOWER_ELEVATOR_MOTOR.setIdleMode(ANGLE_NEUTRAL_MODE_VALUE);

        ELEVATOR_MAG_ENCODER.setSelectedSensorPosition(ELEVATOR_MAG_ENCODER.getSelectedSensorPosition() - ELEVATOR_MAG_ENCODER_OFFSET);
        ELEVATOR_MAG_ENCODER.setSensorPhase(ELEVATOR_MAG_ENCODER_PHASE_VALUE);

    }

    private static void configureElevatorEncoder() {
        ELEVATOR_MAG_ENCODER.setSelectedSensorPosition(ELEVATOR_MAG_ENCODER.getSelectedSensorPosition() - ELEVATOR_MAG_ENCODER_OFFSET);
        ELEVATOR_MAG_ENCODER.setSensorPhase(ELEVATOR_MAG_ENCODER_PHASE_VALUE);
    }


    private static void motors_angle_config() {
        MASTER_ANGLE_MOTOR.restoreFactoryDefaults();
        FOLLOWER_ANGLE_MOTOR.restoreFactoryDefaults();

        MASTER_ANGLE_MOTOR.setInverted(MASTER_ANGLE_INVERTED_VALUE);
        FOLLOWER_ANGLE_MOTOR.setInverted(FOLLOWER_ANGLE_INVERTED_VALUE);

        MASTER_ANGLE_MOTOR.setIdleMode(ANGLE_NEUTRAL_MODE_VALUE);
        FOLLOWER_ANGLE_MOTOR.setIdleMode(ANGLE_NEUTRAL_MODE_VALUE);

        CANcoderConfiguration configureAngleEncoder = new CANcoderConfiguration();
        configureAngleEncoder.MagnetSensor.MagnetOffset = ANGLE_CANCODER_OFFSET;
        configureAngleEncoder.MagnetSensor.AbsoluteSensorRange = ANGLE_ENCODER_SENSOR_RANGE;
        configureAngleEncoder.MagnetSensor.SensorDirection = ANGLE_CANCODER_SENSOR_DIRECTION;
        ANGLE_CANCODER.getConfigurator().apply(configureAngleEncoder);
    }


    public enum ArmStates {
        FIRST_STATE(new Rotation2d(30), 1),
        SECOND_STATE(new Rotation2d(50), 2),
        THIRD_STATE(new Rotation2d(100), 3);
        final Rotation2d angle;

        final double elevatorPosition;

        ArmStates(Rotation2d angle, double elevatorPosition) {
            this.elevatorPosition = elevatorPosition;
            this.angle = angle;
        }

    }

}
