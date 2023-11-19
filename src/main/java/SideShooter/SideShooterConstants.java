package SideShooter;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SideShooterConstants {
    private static final int SHOOTING_MOTOR_ID = 0;
    private static final int ANGLE_MOTOR_ID = 0;
    private static final int ENCODER_ID = 0;
    private static final double MAGNET_OFFSET = 0;
    private static final CANSparkMax.IdleMode ANGLE_MOTOR_IDLE_MODE = CANSparkMax.IdleMode.kBrake;
    private static final AbsoluteSensorRangeValue ANGEL_ENCODER_VALUE = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    private static final SensorDirectionValue ENCODER_SENSOR_DIRECTION = SensorDirectionValue.CounterClockwise_Positive;
    private static final InvertedValue SHOOTER_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    private static final double ANGLE_MOTOR_ENABLE_VOLTAGE_COMPENEATION = 16.45;
    private static final boolean ANGLE_MOTOR_INVERTED = false;
    static final TalonFX SHOOTING_MOTOR = new TalonFX(SHOOTING_MOTOR_ID);
    static final CANSparkMax ANGLE_MOTOR = new CANSparkMax(ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private static final CANcoder ANGLE_ENCODER = new CANcoder(ENCODER_ID);

    private static final double
            MAX_ANGEL_VELOCITY = 600,
            MAX_ANGEL_ACCELERATION = 500;
    static final TrapezoidProfile.Constraints ANGLE_Constraints = new TrapezoidProfile.Constraints(
            MAX_ANGEL_VELOCITY, MAX_ANGEL_ACCELERATION
    );

    private static final int
            ANGLE_MOTOR_P = 0,
            ANGLE_MOTOR_I = 0,
            ANGLE_MOTOR_D = 0;

    static final PIDController ANGLE_PID_CONTROLLER = new PIDController(ANGLE_MOTOR_P, ANGLE_MOTOR_I, ANGLE_MOTOR_D);

    static {
        configureAngleEncoder();
        configureAngleMotor();
        configureShootingMotor();
    }

    private static void configureShootingMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Audio.BeepOnBoot = true;
        config.MotorOutput.Inverted = SHOOTER_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;
        config.Audio.BeepOnConfig = false;
        SHOOTING_MOTOR.getConfigurator().apply(config);
    }

    private static void configureAngleMotor() {
        ANGLE_MOTOR.restoreFactoryDefaults();
        ANGLE_MOTOR.setInverted(ANGLE_MOTOR_INVERTED);
        ANGLE_MOTOR.setIdleMode(ANGLE_MOTOR_IDLE_MODE);
        ANGLE_MOTOR.enableVoltageCompensation(ANGLE_MOTOR_ENABLE_VOLTAGE_COMPENEATION);
    }

    private static void configureAngleEncoder() {
        CANcoderConfiguration configureAngleMotor = new CANcoderConfiguration();
        configureAngleMotor.MagnetSensor.AbsoluteSensorRange = ANGEL_ENCODER_VALUE;
        configureAngleMotor.MagnetSensor.MagnetOffset = MAGNET_OFFSET;
        configureAngleMotor.MagnetSensor.SensorDirection = ENCODER_SENSOR_DIRECTION;
        ANGLE_ENCODER.getConfigurator().apply(configureAngleMotor);
    }

    public enum SideShooterState {
        COLLECT_POSITION(Rotation2d.fromDegrees(0)),
        MID_LEVEL_POSITION(Rotation2d.fromDegrees(222)),
        HIGH_LEVEL_POSITION(Rotation2d.fromDegrees(666.3));
        private Rotation2d angel;

        SideShooterState(Rotation2d angel) {
            this.angel = angel;


        }
    }
}