package frc.trigon.robot.subsystems.turret.toohardturret;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.controller.ArmFeedforward;

public class ToohardTurretConstants {
    private static final int
            MOTOR_ID = 0,
            ENCODER_ID = 0;
    private static final InvertedValue INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    private static final double ENCODER_OFFSET = 0;
    private static final SensorDirectionValue ENCODER_DIRECTION_VALUE = SensorDirectionValue.Clockwise_Positive;
    private static final AbsoluteSensorRangeValue ENCODER_SENSOR_RANGE_VALUE = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    private static final double
            P = 0,
            I = 0,
            D = 0;

    private static final double
            KS = 0,
            KG = 0,
            KV = 0,
            KA = 0;
    static final TalonFX MOTOR = new TalonFX(MOTOR_ID);
    static final CANcoder ENCODER = new CANcoder(ENCODER_ID);

    static final StatusSignal<Double>
            ENCODER_POSITION_SIGNAL = ENCODER.getPosition(),
            ENCODER_VELOCITY_SIGNAL = ENCODER.getVelocity();

    static {
        configureEncoder();
        configureMotor();
    }

    private static void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = INVERTED_VALUE;
        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;
        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;
        config.Slot0.kP = P;
        config.Slot0.kI = I;
        config.Slot0.kD = D;
        config.Slot0.kS = KS;
        config.Slot0.kG = KG;
        config.Slot0.kV = KV;
        config.Slot0.kA = KA;
        config.Feedback.FeedbackRemoteSensorID = ENCODER_ID;
        MOTOR.getConfigurator().apply(config);
    }

    private static void configureEncoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = ENCODER_OFFSET;
        config.MagnetSensor.SensorDirection = ENCODER_DIRECTION_VALUE;
        config.MagnetSensor.AbsoluteSensorRange = ENCODER_SENSOR_RANGE_VALUE;
        ENCODER.getConfigurator().apply(config);
    }
}
