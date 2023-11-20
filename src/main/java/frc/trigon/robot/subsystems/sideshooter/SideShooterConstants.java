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
    private static final int SHOOTING_MOTOR_ID = 0;
    private static final int ANGEL_MOTOR_ID = 0;
    private static final int ENCODER_ID = 0;
    private static final NeutralModeValue SHOOTING_NEUTRAL_MODE_VALUE = NeutralModeValue.Coast;
    private static final CANSparkMax.IdleMode ANGLE_NEUTRAL_MODE_VALUE = CANSparkMax.IdleMode.kBrake;
    private static final InvertedValue SHOOTING_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    private static final boolean ANGLE_INVERTED_VALUE = false;
    private static final double ENCODER_OFFSET = 0;
    private static final AbsoluteSensorRangeValue ENCODER_RANGE_VALUE = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    static final CANcoder ENCODER = new CANcoder(ENCODER_ID);
    static final TalonFX SHOOTING_MOTOR = new TalonFX(SHOOTING_MOTOR_ID);
    static final CANSparkMax ANGEL_MOTOR = new CANSparkMax(ANGEL_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

    static {
        configureShootingMotor();
        configureAngleMotor();
        configureEncoder();
    }
    private static void configureShootingMotor() {
        TalonFXConfiguration ShootingMotorConfig = new TalonFXConfiguration();
        ShootingMotorConfig.Audio.BeepOnBoot = false;
        ShootingMotorConfig.Audio.BeepOnConfig = false;
        ShootingMotorConfig.MotorOutput.NeutralMode = SHOOTING_NEUTRAL_MODE_VALUE;
        ShootingMotorConfig.MotorOutput.Inverted = SHOOTING_INVERTED_VALUE;
        SHOOTING_MOTOR.getConfigurator().apply(ShootingMotorConfig);
    }

    private static void configureAngleMotor(){
        ANGEL_MOTOR.restoreFactoryDefaults();
        ANGEL_MOTOR.setIdleMode(ANGLE_NEUTRAL_MODE_VALUE);
        ANGEL_MOTOR.setInverted(ANGLE_INVERTED_VALUE);
    }

    private static void configureEncoder(){
        CANcoderConfiguration EncoderConfiguration = new CANcoderConfiguration();
        EncoderConfiguration.MagnetSensor.MagnetOffset = ENCODER_OFFSET;
        EncoderConfiguration.MagnetSensor.AbsoluteSensorRange = ENCODER_RANGE_VALUE;
        ENCODER.getConfigurator().apply(EncoderConfiguration);
    }

}
