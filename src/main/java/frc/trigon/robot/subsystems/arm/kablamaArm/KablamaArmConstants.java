package frc.trigon.robot.subsystems.arm.kablamaArm;

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
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import frc.trigon.robot.utilities.Conversions;

public class KablamaArmConstants {
    private static final int
            MASTER_ANGLE_MOTOR_ID = 1,
            MASTER_ELEVATOR_MOTOR_ID = 2,
            FOLLOWER_ANGLE_MOTOR_ID = 3,
            FOLLOWER_ELEVATOR_MOTOR_ID = 4,
            ANGLE_ENCODER_ID = 5,
            ELEVATOR_ENCODER_ID = 6;
    private static final CANSparkMax.IdleMode
            ELEVATOR_IDLE_MODE = CANSparkMax.IdleMode.kBrake,
            ANGLE_IDLE_MODE = CANSparkMax.IdleMode.kBrake;
    private static final int VOLTAGE_COMPENSATION_SATURATION = 12;
    private static final boolean
            MASTER_ANGLE_INVERTED = false,
            MASTER_ELEVATOR_INVERTED = false,
            FOLLOWER_ANGLE_INVERTED = false,
            FOLLOWER_ELEVATOR_INVERTED = false;
    private static final SensorDirectionValue ANGLE_ENCODER_DIRECTION = SensorDirectionValue.CounterClockwise_Positive;
    private static final boolean ELEVATOR_ENCODER_PHASE = false;
    private static final int
            ELEVATOR_ENCODER_OFFSET = 0,
            ANGLE_ENCODER_OFFSET = 0;
    private static final AbsoluteSensorRangeValue ANGLE_ENCODER_RANGE = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

    static final CANcoder ANGLE_ENCODER = new CANcoder(ANGLE_ENCODER_ID);
    static final TalonSRX ELEVATOR_ENCODER = new TalonSRX(ELEVATOR_ENCODER_ID);
    static final CANSparkMax
            MASTER_ANGLE_MOTOR = new CANSparkMax(MASTER_ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
            MASTER_ELEVATOR_MOTOR = new CANSparkMax(MASTER_ELEVATOR_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
            FOLLOWER_ANGLE_MOTOR = new CANSparkMax(FOLLOWER_ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
            FOLLOWER_ELEVATOR_MOTOR = new CANSparkMax(FOLLOWER_ELEVATOR_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

    static final StatusSignal<Double>
            ANGLE_MOTOR_POSITION_SIGNAL = ANGLE_ENCODER.getPosition(),
            ANGLE_MOTOR_VELOCITY_SIGNAL = ANGLE_ENCODER.getVelocity();

    static final double
            ANGLE_P = 1,
            ANGLE_I = 0,
            ANGLE_D = 0,
            ELEVATOR_P = 1,
            ELEVATOR_I = 0,
            ELEVATOR_D = 0;
    public static final PIDController
            ANGLE_PID_CONTROLLER = new PIDController(ANGLE_P, ANGLE_I, ANGLE_D),
            ELEVATOR_PID_CONTROLLER = new PIDController(ELEVATOR_P, ELEVATOR_I, ELEVATOR_D);

    private static final double
            ANGLE_MOTOR_KS = 0,
            ANGLE_MOTOR_KV = 0,
            ANGLE_MOTOR_KA = 0,
            ANGLE_MOTOR_KG = 0,
            ELEVATOR_MOTOR_KS = 0,
            ELEVATOR_MOTOR_KV = 0,
            ELEVATOR_MOTOR_KA = 0,
            ELEVATOR_MOTOR_KG = 0;
    public static final ArmFeedforward ANGLE_FEEDFORWARD = new ArmFeedforward(
            ANGLE_MOTOR_KS, ANGLE_MOTOR_KG, ANGLE_MOTOR_KV, ANGLE_MOTOR_KA
    );
    public static final ElevatorFeedforward ELEVATOR_FEEDFORWARD = new ElevatorFeedforward(
            ELEVATOR_MOTOR_KS, ELEVATOR_MOTOR_KG, ELEVATOR_MOTOR_KV, ELEVATOR_MOTOR_KA
    );


    static {
        configureAngleEncoder();
        configureElevatorEncoder();
        configureElevatorMotors();
        configureAngleMotors();
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

    private static void configureAngleEncoder() {
        CANcoderConfiguration angleEncoderConfig = new CANcoderConfiguration();
        angleEncoderConfig.MagnetSensor.MagnetOffset = ANGLE_ENCODER_OFFSET;
        angleEncoderConfig.MagnetSensor.AbsoluteSensorRange = ANGLE_ENCODER_RANGE;
        angleEncoderConfig.MagnetSensor.SensorDirection = ANGLE_ENCODER_DIRECTION;
        ANGLE_ENCODER.getConfigurator().apply(angleEncoderConfig);

        ANGLE_MOTOR_POSITION_SIGNAL.setUpdateFrequency(100);
        ANGLE_MOTOR_VELOCITY_SIGNAL.setUpdateFrequency(100);
        ANGLE_ENCODER.optimizeBusUtilization();
    }

    private static void configureElevatorEncoder() {
        ELEVATOR_ENCODER.configFactoryDefault();
        ELEVATOR_ENCODER.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
        ELEVATOR_ENCODER.setSensorPhase(ELEVATOR_ENCODER_PHASE);
        ELEVATOR_ENCODER.setSelectedSensorPosition(Conversions.offsetRead(ELEVATOR_ENCODER.getSelectedSensorPosition(), ELEVATOR_ENCODER_OFFSET));
    }
}
