package frc.trigon.robot.subsystems.turret.kablamaturret;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

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
    private static final FeedbackSensorSourceValue ENCODER_TYPE = FeedbackSensorSourceValue.RemoteCANcoder;
    private static final double
            MOTION_MAGIC_VELOCITY = 80,
            MOTION_MAGIC_ACCELERATION = 160,
            MOTION_MAGIC_JERK = 1600;
    private static final double
            P = 1,
            I = 0,
            D = 0,
            KA = 0,
            KG = 1,
            KS = 0,
            KV = 0;
    private static final CANcoder ENCODER = new CANcoder(ENCODER_ID);
    static final TalonFX MOTOR = new TalonFX(MOTOR_ID);

    static final StatusSignal<Double>
            TURRET_POSITION_SIGNAL = ENCODER.getPosition(),
            TURRET_VELOCITY_SIGNAL = ENCODER.getVelocity();

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
        config.Slot0.kA = KA;
        config.Slot0.kG = KG;
        config.Slot0.kS = KS;
        config.Slot0.kV = KV;
        
        config.Feedback.FeedbackRemoteSensorID = ENCODER_ID;
        config.Feedback.FeedbackRotorOffset = ENCODER_OFFSET;
        config.Feedback.FeedbackSensorSource = ENCODER_TYPE;

        config.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = MOTION_MAGIC_JERK;

        MOTOR.getConfigurator().apply(config);
        MOTOR.optimizeBusUtilization();
    }

    private static void configureEncoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.AbsoluteSensorRange = ENCODER_RANGE;
        config.MagnetSensor.SensorDirection = ENCODER_DIRECTION;
        config.MagnetSensor.MagnetOffset = ENCODER_OFFSET;
        ENCODER.getConfigurator().apply(config);

        TURRET_POSITION_SIGNAL.setUpdateFrequency(100);
        TURRET_VELOCITY_SIGNAL.setUpdateFrequency(100);
        ENCODER.optimizeBusUtilization();
    }
}
