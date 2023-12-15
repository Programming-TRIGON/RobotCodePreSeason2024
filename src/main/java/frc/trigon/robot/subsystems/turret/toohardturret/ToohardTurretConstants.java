package frc.trigon.robot.subsystems.turret.toohardturret;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class ToohardTurretConstants {
    private static final int
            MOTOR_ID = 0,
            ENCODER_ID = 0;
    private static final TalonFX MOTOR = new TalonFX(MOTOR_ID);
    private static final CANcoder CANCODER = new CANcoder(ENCODER_ID);
    private static final InvertedValue INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    private static final double ENCODER_OFFSET = 0;
    private static final SensorDirectionValue ENCODER_DIRECTION_VALUE = SensorDirectionValue.Clockwise_Positive;
    private static final AbsoluteSensorRangeValue ENCODER_SENSOR_RANGE_VALUE = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

    static {
        configureMotor();
        configureEncoder();
    }

    private static void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = INVERTED_VALUE;
        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;
        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;
        MOTOR.getConfigurator().apply(config);
    }

    private static void configureEncoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = ENCODER_OFFSET;
        config.MagnetSensor.SensorDirection = ENCODER_DIRECTION_VALUE;
        config.MagnetSensor.AbsoluteSensorRange = ENCODER_SENSOR_RANGE_VALUE;
        CANCODER.getConfigurator().apply(config);
    }
}
