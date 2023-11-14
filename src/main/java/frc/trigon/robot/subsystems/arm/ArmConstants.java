package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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

public class ArmConstants {
    private static final double VOLTAGE_COMPENSATION_SATURATION = 12;

    private static final int
            MASTER_ANGLE_MOTOR_ID = 0,
            FOLLOWER_ANGLE_MOTOR_ID = 1,
            MASTER_ELEVATOR_MOTOR_ID = 2,
            FOLLOWER_ELEVATOR_MOTOR_ID = 3,
            ELEVATOR_ENCODER_ID = 4,
            ANGLE_ENCODER_ID = 5;

    private static final double
            ELEVATOR_ENCODER_OFFSET = 0.0,
            ANGLE_ENCODER_OFFSET = 0.0;

    private static final boolean
            MASTER_ANGLE_INVERTED = false,
            FOLLOWER_ANGLE_INVERTED = false,
            MASTER_ELEVATOR_INVERTED = false,
            FOLLOWER_ELEVATOR_INVERTED = true,
            ELEVATOR_ENCODER_PHASE = true;
    private static final SensorDirectionValue ANGLE_ENCODER_SENSOR_DIRECTION = SensorDirectionValue.Clockwise_Positive;

    private static final double
            MAX_ANGLE_VELOCITY = 100,
            MAX_ANGLE_ACCELERATION = 100,
            MAX_ELEVATOR_VELOCITY = 100,
            MAX_ELEVATOR_ACCELERATION = 100;
    static final TrapezoidProfile.Constraints
            ANGLE_CONSTRAINS = new TrapezoidProfile.Constraints(MAX_ANGLE_VELOCITY, MAX_ANGLE_ACCELERATION),
            ELEVATOR_CONSTRAINS = new TrapezoidProfile.Constraints(MAX_ELEVATOR_VELOCITY, MAX_ELEVATOR_ACCELERATION);

    private static final double
            ANGLE_MOTOR_KS = 0,
            ANGLE_MOTOR_KV = 0,
            ANGLE_MOTOR_KA = 0,
            ANGLE_MOTOR_KG = 0,
            ELEVATOR_MOTOR_KS = 0,
            ELEVATOR_MOTOR_KV = 0,
            ELEVATOR_MOTOR_KA = 0,
            ELEVATOR_MOTOR_KG = 0;
    static final ArmFeedforward ANGLE_MOTOR_FEEDFORWARD = new ArmFeedforward(ANGLE_MOTOR_KS,ANGLE_MOTOR_KG,ANGLE_MOTOR_KV,ANGLE_MOTOR_KA);
    static final ElevatorFeedforward ELEVATOR_MOTOR_FEEDFORWARD = new ElevatorFeedforward(ELEVATOR_MOTOR_KS,ELEVATOR_MOTOR_KG,ELEVATOR_MOTOR_KV,ELEVATOR_MOTOR_KA);

    private static final CANSparkMax.IdleMode
            ANGLE_IDLE_MODE = CANSparkMax.IdleMode.kBrake,
            ELEVATOR_IDLE_MODE = CANSparkMax.IdleMode.kBrake;
    private static final AbsoluteSensorRangeValue ANGLE_ENCODER_SENSOR_RANGE = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

    static final CANSparkMax
            MASTER_ANGLE_MOTOR = new CANSparkMax(MASTER_ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
            FOLLOWER_ANGLE_MOTOR = new CANSparkMax(FOLLOWER_ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
            MASTER_ELEVATOR_MOTOR = new CANSparkMax(MASTER_ELEVATOR_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
            FOLLOWER_ELEVATOR_MOTOR = new CANSparkMax(FOLLOWER_ELEVATOR_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    static final TalonSRX ELEVATOR_ENCODER = new TalonSRX(ELEVATOR_ENCODER_ID);
    static final CANcoder ANGLE_ENCODER = new CANcoder(ANGLE_ENCODER_ID);

    private static final double
            ELEVATOR_P = 1.32,
            ELEVATOR_I = 3.1,
            ELEVATOR_D = 0,
            ANGLE_P = 1.32,
            ANGLE_I = 3.1,
            ANGLE_D = 0;
    private static final PIDController
            ELEVATOR_PID_CONTROLLER = new PIDController(ELEVATOR_P, ELEVATOR_I, ELEVATOR_D),
            ANGLE_PID_CONTROLLER = new PIDController(ANGLE_P, ANGLE_I, ANGLE_D);

    static {
        configureAngleMotors();
        configureElevatorMotors();
        configureElevatorEncoder();
        configureAngleEncoder();
    }

    private static void configureElevatorMotors() {
        MASTER_ELEVATOR_MOTOR.restoreFactoryDefaults();
        FOLLOWER_ELEVATOR_MOTOR.restoreFactoryDefaults();

        MASTER_ELEVATOR_MOTOR.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);
        FOLLOWER_ELEVATOR_MOTOR.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);

        MASTER_ELEVATOR_MOTOR.setInverted(MASTER_ELEVATOR_INVERTED);
        FOLLOWER_ELEVATOR_MOTOR.setInverted(FOLLOWER_ELEVATOR_INVERTED);

        MASTER_ELEVATOR_MOTOR.setIdleMode(ELEVATOR_IDLE_MODE);
        FOLLOWER_ELEVATOR_MOTOR.setIdleMode(ELEVATOR_IDLE_MODE);
    }

    private static void configureElevatorEncoder() {
        ELEVATOR_ENCODER.configFactoryDefault();
        ELEVATOR_ENCODER.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
        ELEVATOR_ENCODER.setSelectedSensorPosition(ELEVATOR_ENCODER.getSelectedSensorPosition() - ELEVATOR_ENCODER_OFFSET);
        ELEVATOR_ENCODER.setSensorPhase(ELEVATOR_ENCODER_PHASE);
    }

    private static void configureAngleMotors() {
        MASTER_ANGLE_MOTOR.restoreFactoryDefaults();
        FOLLOWER_ANGLE_MOTOR.restoreFactoryDefaults();

        MASTER_ANGLE_MOTOR.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);
        FOLLOWER_ANGLE_MOTOR.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);

        MASTER_ANGLE_MOTOR.setInverted(MASTER_ANGLE_INVERTED);
        FOLLOWER_ANGLE_MOTOR.setInverted(FOLLOWER_ANGLE_INVERTED);

        MASTER_ANGLE_MOTOR.setIdleMode(ANGLE_IDLE_MODE);
        FOLLOWER_ANGLE_MOTOR.setIdleMode(ANGLE_IDLE_MODE);
    }

    private static void configureAngleEncoder() {
        CANcoderConfiguration configureAngleEncoder = new CANcoderConfiguration();
        configureAngleEncoder.MagnetSensor.MagnetOffset = ANGLE_ENCODER_OFFSET;
        configureAngleEncoder.MagnetSensor.AbsoluteSensorRange = ANGLE_ENCODER_SENSOR_RANGE;
        configureAngleEncoder.MagnetSensor.SensorDirection = ANGLE_ENCODER_SENSOR_DIRECTION;
        ANGLE_ENCODER.getConfigurator().apply(configureAngleEncoder);
    }

    public enum ArmStates {
        FIRST_STATE(Rotation2d.fromDegrees(30), 1),
        SECOND_STATE(Rotation2d.fromDegrees(50), 2),
        THIRD_STATE(Rotation2d.fromDegrees(100), 3);

        final Rotation2d angle;
        final double elevatorPosition;

        ArmStates(Rotation2d angle, double elevatorPosition) {
            this.elevatorPosition = elevatorPosition;
            this.angle = angle;
        }

    }

}
