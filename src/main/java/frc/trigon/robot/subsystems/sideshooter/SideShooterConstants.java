package frc.trigon.robot.subsystems.sideshooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SideShooterConstants {
    private static final int
            SHOOTING_MOTOR_ID = 0,
            ANGLE_MOTOR_ID = 0,
            ANGLE_ENCODER_ID = 0;
    private static final NeutralModeValue SHOOTING_NEUTRAL_MODE_VALUE = NeutralModeValue.Coast;
    private static final CANSparkMax.IdleMode ANGLE_IDLE_MODE = CANSparkMax.IdleMode.kBrake;
    private static final InvertedValue SHOOTING_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    private static final boolean ANGLE_INVERTED = false;
    private static final SensorDirectionValue ANGLE_ENCODER_SENSOR_DIRECTION_VALUE = SensorDirectionValue.CounterClockwise_Positive;
    private static final double ENCODER_OFFSET = 0;
    private static final AbsoluteSensorRangeValue ENCODER_RANGE_VALUE = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    private static final double VOLTAGE_COMPENSATION_SATURATION = 12;
    static final boolean FOC_ENABLED = true;
    static final CANcoder ANGLE_ENCODER = new CANcoder(ANGLE_ENCODER_ID);
    static final TalonFX SHOOTING_MOTOR = new TalonFX(SHOOTING_MOTOR_ID);
    static final CANSparkMax ANGLE_MOTOR = new CANSparkMax(ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    static final double ANGLE_TOLERANCE = 1;

    private static final double
            MAX_ANGLE_VELOCITY = 0,
            MAX_ANGLE_ACCELERATION = 0;
    static final TrapezoidProfile.Constraints ANGLE_CONSTRAINTS = new TrapezoidProfile.Constraints(
            MAX_ANGLE_VELOCITY,
            MAX_ANGLE_ACCELERATION
    );

    static final StatusSignal<Double>
            ANGLE_ENCODER_POSITION_SIGNAL = ANGLE_ENCODER.getPosition(),
            ANGLE_ENCODER_VELOCITY_SIGNAL = ANGLE_ENCODER.getVelocity();

    private static final double
            ANGLE_P = 0,
            ANGLE_I = 0,
            ANGLE_D = 0;
    static final PIDController ANGLE_PID_CONTROLLER = new PIDController(
            ANGLE_P,
            ANGLE_I,
            ANGLE_D
    );

    private static final double
            ANGLE_MOTOR_KS = 0,
            ANGLE_MOTOR_KV = 0,
            ANGLE_MOTOR_KA = 0,
            ANGLE_MOTOR_KG = 0;
    static final ArmFeedforward ANGLE_MOTOR_FEEDFORWARD = new ArmFeedforward(
            ANGLE_MOTOR_KS,
            ANGLE_MOTOR_KV,
            ANGLE_MOTOR_KA,
            ANGLE_MOTOR_KG
    );

    static {
        configureShootingMotor();
        configureAngleMotor();
        configureAngleEncoder();
    }

    private static void configureShootingMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;
        config.MotorOutput.NeutralMode = SHOOTING_NEUTRAL_MODE_VALUE;
        config.MotorOutput.Inverted = SHOOTING_INVERTED_VALUE;
        SHOOTING_MOTOR.getConfigurator().apply(config);
        SHOOTING_MOTOR.optimizeBusUtilization();
    }

    private static void configureAngleMotor() {
        ANGLE_MOTOR.restoreFactoryDefaults();
        ANGLE_MOTOR.setIdleMode(ANGLE_IDLE_MODE);
        ANGLE_MOTOR.setInverted(ANGLE_INVERTED);
        ANGLE_MOTOR.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);
    }

    private static void configureAngleEncoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = ENCODER_OFFSET;
        config.MagnetSensor.AbsoluteSensorRange = ENCODER_RANGE_VALUE;
        config.MagnetSensor.SensorDirection = ANGLE_ENCODER_SENSOR_DIRECTION_VALUE;
        ANGLE_ENCODER.getConfigurator().apply(config);

        ANGLE_ENCODER_POSITION_SIGNAL.setUpdateFrequency(100);
        ANGLE_ENCODER_VELOCITY_SIGNAL.setUpdateFrequency(100);
        ANGLE_ENCODER.optimizeBusUtilization();
    }

    public enum SideShooterState {
        COLLECTION(Rotation2d.fromDegrees(0), 0),
        HIGH_STATE(Rotation2d.fromDegrees(0), 0),
        MID_STATE(Rotation2d.fromDegrees(0), 0),
        LOW_STATE(Rotation2d.fromDegrees(0), 0);

        final Rotation2d angle;
        final double voltage;

        SideShooterState(Rotation2d angle, double voltage) {
            this.angle = angle;
            this.voltage = voltage;
        }
    }
}