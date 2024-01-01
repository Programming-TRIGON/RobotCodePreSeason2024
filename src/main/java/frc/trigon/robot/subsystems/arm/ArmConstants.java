package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.trigon.robot.utilities.Conversions;

public class ArmConstants {
    private static final int
            MASTER_ELEVATOR_MOTOR_ID = 0,
            FOLLOWER_ELEVATOR_MOTOR_ID = 0,
            MASTER_ANGLE_MOTOR_ID = 0,
            FOLLOWER_ANGLE_MOTOR_ID = 0,
            ANGLE_ENCODER_ID = 0,
            ELEVATOR_ENCODER_ID = 0;
    private static final CANSparkMax.IdleMode
            ANGLE_IDLE_MODE = CANSparkMax.IdleMode.kBrake,
            ELEVATOR_IDLE_MODE = CANSparkMax.IdleMode.kBrake;
    private static final boolean
            MASTER_ANGLE_INVERTED = false,
            FOLLOWER_ANGLE_INVERTED = false,
            MASTER_ELEVATOR_INVERTED = false,
            FOLLOWER_ELEVATOR_INVERTED = false,
            ELEVATOR_ENCODER_PHASE = false;
    private static final SensorDirectionValue ANGLE_ENCODER_SENSOR_DIRECTION_VALUE = SensorDirectionValue.CounterClockwise_Positive;
    private static final double VOLTAGE_COMPENSATION_SATURATION = 12;
    private static final double
            ANGLE_ENCODER_OFFSET = 0,
            ELEVATOR_ENCODER_OFFSET = 0;
    private static final AbsoluteSensorRangeValue ANGLE_ENCODER_RANGE_VALUE = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    static final CANSparkMax
            MASTER_ELEVATOR_MOTOR = new CANSparkMax(MASTER_ELEVATOR_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
            FOLLOWER_ELEVATOR_MOTOR = new CANSparkMax(FOLLOWER_ELEVATOR_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
            MASTER_ANGLE_MOTOR = new CANSparkMax(MASTER_ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
            FOLLOWER_ANGLE_MOTOR = new CANSparkMax(FOLLOWER_ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    static final CANcoder ANGLE_ENCODER = new CANcoder(ANGLE_ENCODER_ID);
    static final WPI_TalonSRX ELEVATOR_ENCODER = new WPI_TalonSRX(ELEVATOR_ENCODER_ID);
    static final double ELEVATOR_METERS_PER_REVOLUTION = 1;
    static final double
            ANGLE_TOLERANCE_DEGREES = 1,
            ELEVATOR_TOLERANCE_METERS = 1;

    private static final double
            MAX_ELEVATOR_VELOCITY = 0,
            MAX_ELEVATOR_ACCELERATION = 0,
            MAX_ANGLE_VELOCITY = 0,
            MAX_ANGLE_ACCELERATION = 0;
    static final TrapezoidProfile.Constraints ELEVATOR_CONSTRAINTS = new TrapezoidProfile.Constraints(
            MAX_ELEVATOR_VELOCITY,
            MAX_ELEVATOR_ACCELERATION
    ),
            ANGLE_CONSTRAINTS = new TrapezoidProfile.Constraints(
                    MAX_ANGLE_VELOCITY,
                    MAX_ANGLE_ACCELERATION
            );

    static final StatusSignal<Double>
            ANGLE_ENCODER_POSITION_SIGNAL = ANGLE_ENCODER.getPosition(),
            ANGLE_ENCODER_VELOCITY_SIGNAL = ANGLE_ENCODER.getVelocity();

    private static final double
            ANGLE_P = 0,
            ANGLE_I = 0,
            ANGLE_D = 0,
            ELEVATOR_P = 0,
            ELEVATOR_I = 0,
            ELEVATOR_D = 0;

    static final PIDController
            ANGLE_PID_CONTROLLER = new PIDController(
            ANGLE_P,
            ANGLE_I,
            ANGLE_D
    ),
            ELEVATOR_PID_CONTROLLER = new PIDController(
                    ELEVATOR_P,
                    ELEVATOR_I,
                    ELEVATOR_D
            );

    private static final double
            ANGLE_MOTOR_KS = 0,
            ANGLE_MOTOR_KV = 0,
            ANGLE_MOTOR_KA = 0,
            ANGLE_MOTOR_KG = 0,
            ELEVATOR_MOTOR_KS = 0,
            ELEVATOR_MOTOR_KV = 0,
            ELEVATOR_MOTOR_KA = 0,
            ELEVATOR_MOTOR_KG = 0;
    static final ArmFeedforward
            ANGLE_MOTOR_FEEDFORWARD = new ArmFeedforward(
            ANGLE_MOTOR_KS,
            ANGLE_MOTOR_KV,
            ANGLE_MOTOR_KA,
            ANGLE_MOTOR_KG
    );
    static final ElevatorFeedforward ELEVATOR_MOTOR_FEEDFORWARD = new ElevatorFeedforward(
            ELEVATOR_MOTOR_KS,
            ELEVATOR_MOTOR_KG,
            ELEVATOR_MOTOR_KV,
            ELEVATOR_MOTOR_KA
    );

    static {
        configureAngleMotors();
        configureElevatorMotors();
        configureAngleEncoder();
        configureElevatorEncoder();
    }

    private static void configureAngleMotors() {
        MASTER_ANGLE_MOTOR.restoreFactoryDefaults();
        FOLLOWER_ANGLE_MOTOR.restoreFactoryDefaults();
        MASTER_ANGLE_MOTOR.setIdleMode(ANGLE_IDLE_MODE);
        FOLLOWER_ANGLE_MOTOR.setIdleMode(ANGLE_IDLE_MODE);
        MASTER_ANGLE_MOTOR.setInverted(MASTER_ANGLE_INVERTED);
        FOLLOWER_ANGLE_MOTOR.setInverted(FOLLOWER_ANGLE_INVERTED);
        MASTER_ANGLE_MOTOR.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);
        FOLLOWER_ANGLE_MOTOR.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);
    }

    private static void configureElevatorMotors() {
        MASTER_ELEVATOR_MOTOR.restoreFactoryDefaults();
        FOLLOWER_ELEVATOR_MOTOR.restoreFactoryDefaults();
        MASTER_ELEVATOR_MOTOR.setIdleMode(ELEVATOR_IDLE_MODE);
        FOLLOWER_ELEVATOR_MOTOR.setIdleMode(ELEVATOR_IDLE_MODE);
        MASTER_ELEVATOR_MOTOR.setInverted(MASTER_ELEVATOR_INVERTED);
        FOLLOWER_ELEVATOR_MOTOR.setInverted(FOLLOWER_ELEVATOR_INVERTED);
        MASTER_ELEVATOR_MOTOR.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);
        FOLLOWER_ELEVATOR_MOTOR.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);
    }

    private static void configureAngleEncoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = ANGLE_ENCODER_OFFSET;
        config.MagnetSensor.AbsoluteSensorRange = ANGLE_ENCODER_RANGE_VALUE;
        config.MagnetSensor.SensorDirection = ANGLE_ENCODER_SENSOR_DIRECTION_VALUE;
        ANGLE_ENCODER.getConfigurator().apply(config);

        ANGLE_ENCODER_POSITION_SIGNAL.setUpdateFrequency(100);
        ANGLE_ENCODER_VELOCITY_SIGNAL.setUpdateFrequency(100);
        ANGLE_ENCODER.optimizeBusUtilization();
    }

    private static void configureElevatorEncoder() {
        ELEVATOR_ENCODER.configFactoryDefault();
        ELEVATOR_ENCODER.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        ELEVATOR_ENCODER.setSensorPhase(ELEVATOR_ENCODER_PHASE);
        ELEVATOR_ENCODER.setSelectedSensorPosition(Conversions.offsetRead(ELEVATOR_ENCODER.getSelectedSensorPosition(), ELEVATOR_ENCODER_OFFSET));
    }

    public enum ArmState {
        CONE_COLLECTION(Rotation2d.fromDegrees(0), 0),
        CONE_HIGH_STATE(Rotation2d.fromDegrees(0), 0),
        CONE_MID_STATE(Rotation2d.fromDegrees(0), 0),
        CONE_LOW_STATE(Rotation2d.fromDegrees(0), 0);

        final Rotation2d angle;
        final double elevatorPositionMeters;

        ArmState(Rotation2d angle, double elevatorPositionMeters) {
            this.angle = angle;
            this.elevatorPositionMeters = elevatorPositionMeters;
        }
    }
}
