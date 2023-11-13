package frc.trigon.robot.subsystems.sideshooter;

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

public class SideShooterConstants {

    private static final double VOLTAGE_COMPENSATION = 12;
    private static final int
            SHOOTING_MOTOR_ID = 0,
            ANGLE_MOTOR_ID = 1,
            ANGLE_ENCODER_ID = 2;

    static final CANSparkMax ANGLE_MOTOR = new CANSparkMax(ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    static final CANcoder ANGLE_ENCODER = new CANcoder(ANGLE_ENCODER_ID);

    static final TalonFX SHOOTING_MOTOR = new TalonFX(SHOOTING_MOTOR_ID);
    private static final double ANGLE_ENCODER_OFFSET = 0;
    private static final AbsoluteSensorRangeValue ANGLE_ENCODER_RANGE = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    private static final InvertedValue SHOOTING_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    private static final boolean ANGLE_MOTOR_INVERTED = false;
    private static final SensorDirectionValue ANGLE_ENCODER_DIRECTION = SensorDirectionValue.Clockwise_Positive;
    private static final NeutralModeValue SHOOTING_NEUTRAL_MODE_VALUE = NeutralModeValue.Coast;
    private static final CANSparkMax.IdleMode ANGLE_MOTOR_IDLE_MODE = CANSparkMax.IdleMode.kBrake;
    private static final double
            P = 1,
            I = 0,
            D = 0;
    static final PIDController PID_CONTROLLER = new PIDController(P, I, D);

    static {
        configureAngleEncoder();
        configureShootingMotor();
        configureAngleMotor();
    }

    private static void configureShootingMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;
        config.MotorOutput.Inverted = SHOOTING_MOTOR_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = SHOOTING_NEUTRAL_MODE_VALUE;
        SHOOTING_MOTOR.getConfigurator().apply(config);
    }

    private static void configureAngleMotor() {
        ANGLE_MOTOR.restoreFactoryDefaults();
        ANGLE_MOTOR.setIdleMode(ANGLE_MOTOR_IDLE_MODE);
        ANGLE_MOTOR.setInverted(ANGLE_MOTOR_INVERTED);
        ANGLE_MOTOR.enableVoltageCompensation(VOLTAGE_COMPENSATION);
    }

    private static void configureAngleEncoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.AbsoluteSensorRange = ANGLE_ENCODER_RANGE;
        config.MagnetSensor.MagnetOffset = ANGLE_ENCODER_OFFSET;
        config.MagnetSensor.SensorDirection = ANGLE_ENCODER_DIRECTION;
        ANGLE_ENCODER.getConfigurator().apply(config);
    }

    public enum SideShooter {
        COLLECTOR(-15, -5),
        MIDDLE(30, 5),
        HIGH(60, 10);

        final double angle, power;

        SideShooter(double angle, double power) {
            this.angle = angle;
            this.power = power;
        }
    }
}

