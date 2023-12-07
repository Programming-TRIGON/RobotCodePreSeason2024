package frc.trigon.robot.subsystems.turret;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TurretConstants {
    private static final int
            MOTOR_ID = 1,
            ENCODER_ID = 2;
    private static final InvertedValue INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    private static final AbsoluteSensorRangeValue ENCODER_RANGE = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    private static final SensorDirectionValue ENCODER_DIRECTION = SensorDirectionValue.CounterClockwise_Positive;
    private static final double ENCODER_OFFSET = 0;
    private static final double
            HUB_X = 8.2296,
            HUB_Y = 0.5121;

    private static final Translation2d HUB_POSITION = new Translation2d(HUB_X, HUB_Y);
    private static final Rotation2d HUB_ROTATION = Rotation2d.fromRotations(0);
    static final TalonFX MOTOR = new TalonFX(MOTOR_ID);
    static final CANcoder ENCODER = new CANcoder(ENCODER_ID);
    static final double GEAR_RATIO = 100;
    static final double MOMENT_OF_INERTIA = 0.003;
    static final boolean FOC_ENABLED = true;
    static final StatusSignal<Double> STATUS_SIGNAL = ENCODER.getPosition();
    static final Pose2d HUB_POSE = new Pose2d(HUB_POSITION, HUB_ROTATION);
    static final Pose2d ROBOT_POSE = new Pose2d();

    private static final double
            P = 1,
            I = 0,
            D = 0;
    static final PIDController PID_CONTROLLER = new PIDController(P, I, D);

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
        MOTOR.getConfigurator().apply(config);
    }

    private static void configureEncoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.AbsoluteSensorRange = ENCODER_RANGE;
        config.MagnetSensor.SensorDirection = ENCODER_DIRECTION;
        config.MagnetSensor.MagnetOffset = ENCODER_OFFSET;
        ENCODER.getConfigurator().apply(config);

        STATUS_SIGNAL.setUpdateFrequency(100);
        ENCODER.optimizeBusUtilization();
    }
}
