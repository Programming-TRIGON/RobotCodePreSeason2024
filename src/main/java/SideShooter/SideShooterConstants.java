package SideShooter;

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
    private static final InvertedValue SHOOTER_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    private static final boolean ANGLE_MOTOR_INVERTED = false;
    private static final int ANGLE_MOTOR_ID = 0;
    static final CANSparkMax ANGLE_MOTOR = new CANSparkMax(ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final static int Encoder_ID = 0;
    private static final CANcoder ANGLE_ENCODER = new CANcoder(Encoder_ID);

    private static final double
            MAX_ANGEL_VELOCITY = 600,

            MAX_ANGEL_ACCELERATION = 500;


    private static final int
            ANGLE_MOTOR_P = 0,
            ANGLE_MOTOR_I = 0,
            ANGLE_MOTOR_D = 0;

    static final PIDController ANGLE_PID_CONTROLLER = new PIDController(ANGLE_MOTOR_P, ANGLE_MOTOR_I, ANGLE_MOTOR_D);

    static final TalonFX SHOOTING_MOTOR = new TalonFX(SHOOTING_MOTOR_ID);

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
        ANGLE_PID_CONTROLLER.setSetpoint(0);
        ANGLE_MOTOR.setInverted(ANGLE_MOTOR_INVERTED);
        ANGLE_MOTOR.setIdleMode(CANSparkMax.IdleMode.kBrake);
        ANGLE_MOTOR.enableVoltageCompensation(12);
    }

    private static void configureAngleEncoder() {
        CANcoderConfiguration configureAngleMotor = new CANcoderConfiguration();
        configureAngleMotor.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        configureAngleMotor.MagnetSensor.MagnetOffset = 0;
        configureAngleMotor.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        ANGLE_ENCODER.getConfigurator().apply(configureAngleMotor);
    }

    static final TrapezoidProfile.Constraints ANGLE_Constraints = new TrapezoidProfile.Constraints(
            MAX_ANGEL_VELOCITY, MAX_ANGEL_ACCELERATION
    );

    static {
        configureAngleEncoder();
        configureAngleMotor();
        configureShootingMotor();
    }

    public enum SideShooterState {
        COLLECT_POSITION(Rotation2d.fromDegrees(0)),
        MID_LEVEL_POSITION(Rotation2d.fromDegrees(222)),
        HIGH_LEVEL_POSITION(Rotation2d.fromDegrees(666.3));
        private Rotation2d angel;

        SideShooterState(Rotation2d angel){
            this.angel = angel;


        }
    }
}