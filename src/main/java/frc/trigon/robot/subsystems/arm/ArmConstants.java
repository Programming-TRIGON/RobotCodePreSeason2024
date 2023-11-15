package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.trigon.robot.utilities.Conversions;

public class ArmConstants {
    private static final int
            MASTER_ANGLE_MOTOR_ID = 0,
            FOLLOWER_ANGLE_MOTOR_ID = 1,
            MASTER_ELEVATOR_MOTOR_ID = 0,
            FOLLOWER_ELEVATOR_MOTOR_ID = 1,
            ANGLE_ENCODER_ID = 0,
            ELEVATOR_ENCODER_ID = 1;
    private static final boolean
            MASTER_ANGLE_MOTOR_INVERTED = false,
            FOLLOWER_ANGLE_MOTOR_INVERTED = false,
            MASTER_ELEVATOR_MOTOR_INVERTED = false,
            FOLLOWER_ELEVATOR_MOTOR_INVERTED = false;
    private static final CANSparkMax.IdleMode
            ANGLE_MOTORS_IDLE_MODE = CANSparkMax.IdleMode.kBrake,
            ELEVATOR_MOTORS_IDLE_MODE = CANSparkMax.IdleMode.kBrake;
    private static final int VOLTAGE_COMPENSATION_SATURATION = 12;
    static final CANSparkMax
            MASTER_ANGLE_MOTOR = new CANSparkMax(MASTER_ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
            FOLLOWER_ANGLE_MOTOR = new CANSparkMax(FOLLOWER_ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
            MASTER_ELEVATOR_MOTOR = new CANSparkMax(MASTER_ELEVATOR_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
            FOLLOWER_ELEVATOR_MOTOR = new CANSparkMax(FOLLOWER_ELEVATOR_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

    private static final double
            ANGLE_MOTOR_OFFSET = 0,
            ELEVATOR_MOTOR_OFFSET = 0;
    private static final SensorDirectionValue ANGLE_MOTOR_DIRECTION = SensorDirectionValue.Clockwise_Positive;
    private static final boolean ELEVATOR_MOTOR_PHASE = false;
    private static final AbsoluteSensorRangeValue ANGLE_MOTOR_RANGE = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    static final CANcoder ANGLE_ENCODER = new CANcoder(ANGLE_ENCODER_ID);
    static final TalonSRX ELEVATOR_ENCODER = new TalonSRX(ELEVATOR_ENCODER_ID);

    private static final int
            ANGLE_P = 0,
            ANGLE_I = 0,
            ANGLE_D = 0,
            ELEVATOR_P = 0,
            ELEVATOR_I = 0,
            ELEVATOR_D = 0;
    static final PIDController
            ANGLE_PID_CONTROLLER = new PIDController(ANGLE_P, ANGLE_I, ANGLE_D),
            ELEVATOR_PID_CONTROLLER = new PIDController(ELEVATOR_P, ELEVATOR_I, ELEVATOR_D);

    private static final double
            MAX_ANGLE_VELOCITY = 100,
            MAX_ELEVATOR_VELOCITY = 100,
            MAX_ANGLE_ACCELERATION = 100,
            MAX_ELEVATOR_ACCELERATION = 100;
    static final TrapezoidProfile.Constraints
            ANGLE_CONSTRAINTS = new TrapezoidProfile.Constraints(MAX_ANGLE_VELOCITY, MAX_ANGLE_ACCELERATION),
            ELEVATOR_CONSTRAINTS = new TrapezoidProfile.Constraints(MAX_ELEVATOR_VELOCITY, MAX_ELEVATOR_ACCELERATION);
    private static final double
            ANGLE_MOTOR_KS = 1,
            ANGLE_MOTOR_KG = 1,
            ANGLE_MOTOR_KV = 1,
            ANGLE_MOTOR_KA = 1,
            ELEVATOR_MOTOR_KS = 1,
            ELEVATOR_MOTOR_KG = 1,
            ELEVATOR_MOTOR_KV = 1,
            ELEVATOR_MOTOR_KA = 1;
    static final ArmFeedforward ARM_FEEDFORWARD = new ArmFeedforward(
            ANGLE_MOTOR_KS,
            ANGLE_MOTOR_KG,
            ANGLE_MOTOR_KV,
            ANGLE_MOTOR_KA
    );
    static final ArmFeedforward ANGLE_FEEDFORWARD = new ArmFeedforward(
            ELEVATOR_MOTOR_KS,
            ELEVATOR_MOTOR_KG,
            ELEVATOR_MOTOR_KV,
            ELEVATOR_MOTOR_KA
    );

    static final StatusSignal<Double>
            ANGLE_ENCODER_POSITION_SIGNAL = ANGLE_ENCODER.getPosition(),
            ANGLE_ENCODER_VELOCITY_SIGNAL = ANGLE_ENCODER.getVelocity();

    static {
        configureAngleMotors();
        configureElevatorMotors();
        configureAngleEncoder();
        configureElevatorEncoder();
    }

    private static void configureAngleMotors() {
        MASTER_ANGLE_MOTOR.restoreFactoryDefaults();
        FOLLOWER_ANGLE_MOTOR.restoreFactoryDefaults();
        MASTER_ANGLE_MOTOR.setInverted(MASTER_ANGLE_MOTOR_INVERTED);
        FOLLOWER_ANGLE_MOTOR.setInverted(FOLLOWER_ANGLE_MOTOR_INVERTED);
        MASTER_ANGLE_MOTOR.setIdleMode(ANGLE_MOTORS_IDLE_MODE);
        FOLLOWER_ANGLE_MOTOR.setIdleMode(ANGLE_MOTORS_IDLE_MODE);
        MASTER_ANGLE_MOTOR.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);
        FOLLOWER_ANGLE_MOTOR.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);
    }

    private static void configureElevatorMotors() {
        MASTER_ELEVATOR_MOTOR.restoreFactoryDefaults();
        FOLLOWER_ELEVATOR_MOTOR.restoreFactoryDefaults();
        MASTER_ELEVATOR_MOTOR.setInverted(MASTER_ELEVATOR_MOTOR_INVERTED);
        FOLLOWER_ELEVATOR_MOTOR.setInverted(FOLLOWER_ELEVATOR_MOTOR_INVERTED);
        MASTER_ELEVATOR_MOTOR.setIdleMode(ELEVATOR_MOTORS_IDLE_MODE);
        FOLLOWER_ELEVATOR_MOTOR.setIdleMode(ELEVATOR_MOTORS_IDLE_MODE);
        MASTER_ELEVATOR_MOTOR.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);
        FOLLOWER_ELEVATOR_MOTOR.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);
    }

    private static void configureAngleEncoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = ANGLE_MOTOR_OFFSET;
        config.MagnetSensor.SensorDirection = ANGLE_MOTOR_DIRECTION;
        config.MagnetSensor.AbsoluteSensorRange = ANGLE_MOTOR_RANGE;
        ANGLE_ENCODER.getConfigurator().apply(config);

        ANGLE_ENCODER_POSITION_SIGNAL.setUpdateFrequency(100);
        ANGLE_ENCODER_VELOCITY_SIGNAL.setUpdateFrequency(100);
    }

    private static void configureElevatorEncoder() {
        ELEVATOR_ENCODER.configFactoryDefault();
        ELEVATOR_ENCODER.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        ELEVATOR_ENCODER.setSensorPhase(ELEVATOR_MOTOR_PHASE);
        ELEVATOR_ENCODER.setSelectedSensorPosition(Conversions.offsetRead(ELEVATOR_ENCODER.getSelectedSensorPosition(), ELEVATOR_MOTOR_OFFSET));
    }

    public enum ArmState {
        DEFAULT(Rotation2d.fromDegrees(0), 0),
        TAKE_HIGH_CONE(Rotation2d.fromDegrees(60), 10),
        TAKE_GROUND_CONE(Rotation2d.fromDegrees(0), 0),
        PLACE_LOW_CONE(Rotation2d.fromDegrees(20), 3),
        PLACE_MEDIUM_CONE(Rotation2d.fromDegrees(50), 7),
        PLACE_HIGH_CONE(Rotation2d.fromDegrees(70), 10);

        final Rotation2d angle;
        final double elevatorPosition;

        ArmState(Rotation2d angle, double elevatorPosition) {
            this.angle = angle;
            this.elevatorPosition = elevatorPosition;
        }
    }
}
