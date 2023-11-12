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
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;

import java.time.OffsetDateTime;
import java.time.OffsetTime;

public class SideShooterConstants {


    private static final int MOTOR_SHOOT_ID = 0;
    private static final InvertedValue INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    private static final int MOTOR_angel_ID = 0;
    private static SparkMaxPIDController MOTOR_angel_PIDController;
    static final CANSparkMax MOTOR_angel = new CANSparkMax(MOTOR_angel_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final static int Encoder_ID = 0;
    private static final CANcoder encoder = new CANcoder(Encoder_ID);

    private static final int
    P= 0,
    I = 0,
    D = 0;

    static final PIDController PID_CONTROLLER = new PIDController(P, I, D);

    static final TalonFX MOTOR_SHOOT = new TalonFX(MOTOR_SHOOT_ID);


    static {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Audio.BeepOnBoot = false;
        config.MotorOutput.Inverted = INVERTED_VALUE;
        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;
        config.Audio.BeepOnConfig = false;

        MOTOR_angel.restoreFactoryDefaults();
        MOTOR_angel_PIDController = MOTOR_angel.getPIDController();
        MOTOR_angel.setInverted(false);
        MOTOR_angel.setIdleMode(CANSparkMax.IdleMode.kBrake);
        MOTOR_angel.setVoltage(12);

        MOTOR_SHOOT.getConfigurator().apply(config);
        CANcoderConfiguration configuration = new CANcoderConfiguration();
        configuration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        configuration.MagnetSensor.MagnetOffset = 0;
        configuration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

    }





}
