package frc.trigon.robot.subsystems.sideshooter;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public class SideShooterConstants {
    private static final int
            SHOOTING_MOTOR_ID = 0,
            ANGLE_MOTOR_ID = 0,
            ANGLE_ENCODER_ID = 0;
    private static final NeutralModeValue SHOOTING_NEUTRAL_MODE_VALUE = NeutralModeValue.Coast;
    private static final CANSparkMax.IdleMode ANGLE_IDLE_MODE = CANSparkMax.IdleMode.kBrake;
    private static final InvertedValue SHOOTING_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    private static final boolean ANGLE_INVERTED = false;
    private static final double ENCODER_OFFSET = 0;
    private static final AbsoluteSensorRangeValue ENCODER_RANGE_VALUE = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    private static final double VOLTAGE_COMPENSATION_VALUE = 0;
    static final CANcoder ANGLE_ENCODER = new CANcoder(ANGLE_ENCODER_ID);
    static final TalonFX SHOOTING_MOTOR = new TalonFX(SHOOTING_MOTOR_ID);
    static final CANSparkMax ANGEL_MOTOR = new CANSparkMax(ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

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

    private static void configureAngleMotor(){
        ANGEL_MOTOR.restoreFactoryDefaults();
        ANGEL_MOTOR.setIdleMode(ANGLE_IDLE_MODE);
        ANGEL_MOTOR.setInverted(ANGLE_INVERTED);
        ANGEL_MOTOR.enableVoltageCompensation(VOLTAGE_COMPENSATION_VALUE);
    }

    private static void configureAngleEncoder(){
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = ENCODER_OFFSET;
        config.MagnetSensor.AbsoluteSensorRange = ENCODER_RANGE_VALUE;
        ANGLE_ENCODER.getConfigurator().apply(config);
        ANGLE_ENCODER.optimizeBusUtilization();
    }

}