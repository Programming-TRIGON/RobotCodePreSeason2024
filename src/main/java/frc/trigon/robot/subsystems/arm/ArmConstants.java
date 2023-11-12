package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.utilities.Conversions;

public class ArmConstants {
    private static final int
            MASTER_ANGLE_MOTOR_ID = 1,
            MASTER_ELEVATOR_MOTOR_ID = 2,
            FOLLOWER_ANGLE_MOTOR_ID = 3,
            FOLLOWER_ELEVATOR_MOTOR_ID = 4,
            ANGLE_ENCODER_ID = 5,
            ELEVATOR_ENCODER_ID = 6;
    private static final CANSparkMax.IdleMode
            ELEVATOR_IDLE_MODE = CANSparkMax.IdleMode.kBrake,
            ANGLE_IDLE_MODE = CANSparkMax.IdleMode.kBrake;
    private static final int
            VOLTAGE_COMPANSATION_SATURATION = 12;
    private static final boolean
            MASTER_ANGLE_INVERTED = false,
            MASTER_ELEVATOR_INVERTED = false,
            FOLLOWER_ANGLE_INVERTED = false,
            FOLLOWER_ELEVATOR_INVERTED = false;
    private static final SensorDirectionValue ANGLE_ENCODER_DIRECTION = SensorDirectionValue.CounterClockwise_Positive;
    private static final boolean ELEVATOR_ENCODER_PHASE = false;
    private static final int
            ELEVATOR_ENCODER_OFFSET = 5,
            ANGLE_ENCODER_OFFSET = 5;
    private static final AbsoluteSensorRangeValue ANGLE_ENCODER_RANGE = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    private static final CANSparkMax
            MASTER_ANGLE_MOTOR = new CANSparkMax(MASTER_ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
            MASTER_ELEVATOR_MOTOR = new CANSparkMax(MASTER_ELEVATOR_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
            FOLLOWER_ANGLE_MOTOR = new CANSparkMax(FOLLOWER_ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
            FOLLOWER_ELEVATOR_MOTOR = new CANSparkMax(FOLLOWER_ELEVATOR_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private static final CANcoder ANGLE_ENCODER = new CANcoder(ANGLE_ENCODER_ID);
    private static final TalonSRX ELEVATOR_ENCODER = new TalonSRX(ELEVATOR_ENCODER_ID);
    static final double
            ANGLE_P = 1,
            ANGLE_I = 0,
            ANGLE_D = 0,
            ELEVATOR_P = 1,
            ELEVATOR_I = 0,
            ELEVATOR_D = 0;
    static final PIDController
            ANGLE_PID_CONTROLLER = new PIDController(ANGLE_P, ANGLE_I, ANGLE_D),
            ELEVATOR_PID_CONTROLLER = new PIDController(ELEVATOR_P, ELEVATOR_I, ELEVATOR_D);

    static {
        configureAngleEncoder();
        configureElevatorEncoder();
        configureElevatorMotors();
        configureAngleMotors();
    }

    private static void configureElevatorMotors() {
        MASTER_ELEVATOR_MOTOR.restoreFactoryDefaults();
        FOLLOWER_ELEVATOR_MOTOR.restoreFactoryDefaults();
        MASTER_ELEVATOR_MOTOR.setIdleMode(ELEVATOR_IDLE_MODE);
        FOLLOWER_ELEVATOR_MOTOR.setIdleMode(ELEVATOR_IDLE_MODE);
        MASTER_ELEVATOR_MOTOR.setInverted(MASTER_ELEVATOR_INVERTED);
        FOLLOWER_ELEVATOR_MOTOR.setInverted(FOLLOWER_ELEVATOR_INVERTED);
        MASTER_ELEVATOR_MOTOR.enableVoltageCompensation(VOLTAGE_COMPANSATION_SATURATION);
        FOLLOWER_ELEVATOR_MOTOR.enableVoltageCompensation(VOLTAGE_COMPANSATION_SATURATION);
    }

    private static void configureAngleMotors() {
        MASTER_ANGLE_MOTOR.restoreFactoryDefaults();
        FOLLOWER_ANGLE_MOTOR.restoreFactoryDefaults();
        MASTER_ANGLE_MOTOR.setIdleMode(ANGLE_IDLE_MODE);
        FOLLOWER_ANGLE_MOTOR.setIdleMode(ANGLE_IDLE_MODE);
        MASTER_ANGLE_MOTOR.setInverted(MASTER_ANGLE_INVERTED);
        FOLLOWER_ANGLE_MOTOR.setInverted(FOLLOWER_ANGLE_INVERTED);
        MASTER_ANGLE_MOTOR.enableVoltageCompensation(VOLTAGE_COMPANSATION_SATURATION);
        FOLLOWER_ANGLE_MOTOR.enableVoltageCompensation(VOLTAGE_COMPANSATION_SATURATION);
    }


    private static void configureAngleEncoder() {
        CANcoderConfiguration angleEncoderConfig = new CANcoderConfiguration();
        angleEncoderConfig.MagnetSensor.MagnetOffset = ANGLE_ENCODER_OFFSET;
        angleEncoderConfig.MagnetSensor.AbsoluteSensorRange = ANGLE_ENCODER_RANGE;
        angleEncoderConfig.MagnetSensor.SensorDirection = ANGLE_ENCODER_DIRECTION;
        ANGLE_ENCODER.getConfigurator().apply(angleEncoderConfig);
    }

    private static void configureElevatorEncoder() {
        ELEVATOR_ENCODER.configFactoryDefault();
        ELEVATOR_ENCODER.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        ELEVATOR_ENCODER.setSensorPhase(ELEVATOR_ENCODER_PHASE);
        ELEVATOR_ENCODER.setSelectedSensorPosition(Conversions.offsetRead(ELEVATOR_ENCODER.getSelectedSensorPosition(), ELEVATOR_ENCODER_OFFSET));
    }

    public enum ArmState {
        FIRST_STATE(Rotation2d.fromDegrees(100), 7),
        SECOND_STATE(Rotation2d.fromDegrees(70), 5);

        final Rotation2d angle;
        final double elevatorPosition;

        ArmState(Rotation2d angle, double elevatorPosition) {
            this.angle = angle;
            this.elevatorPosition = elevatorPosition;
        }
    }
}
