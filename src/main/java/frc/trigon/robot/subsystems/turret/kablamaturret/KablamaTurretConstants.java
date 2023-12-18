package frc.trigon.robot.subsystems.turret.kablamaturret;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class KablamaTurretConstants {
    static final boolean FOC_ENABLED = true;
    private static final int
            MOTOR_ID = 1,
            ENCODER_ID = 2;
    private static final InvertedValue INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    private static final AbsoluteSensorRangeValue ENCODER_RANGE = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    private static final SensorDirectionValue ENCODER_DIRECTION = SensorDirectionValue.CounterClockwise_Positive;
    private static final double ENCODER_OFFSET = 0;

    private static final double
            P = 1,
            I = 0,
            D = 0;
    private static final CANcoder ENCODER = new CANcoder(ENCODER_ID);
    static final TalonFX MOTOR = new TalonFX(MOTOR_ID);
    static final StatusSignal<Double> TURRET_POSITION_SIGNAL = ENCODER.getPosition();

    static {
        configureMotor();
        configureEncoder();
    }

    private static void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = INVERTED_VALUE;
        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;
        config.Audio.BeepOnConfig = false;
        config.Audio.BeepOnBoot = false;
        config.Slot0.kP = P;
        config.Slot0.kI = I;
        config.Slot0.kD = D;
        MOTOR.getConfigurator().apply(config);
    }

    private static void configureEncoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.AbsoluteSensorRange = ENCODER_RANGE;
        config.MagnetSensor.SensorDirection = ENCODER_DIRECTION;
        config.MagnetSensor.MagnetOffset = ENCODER_OFFSET;
        ENCODER.getConfigurator().apply(config);

        TURRET_POSITION_SIGNAL.setUpdateFrequency(100);
        ENCODER.optimizeBusUtilization();
    }
}