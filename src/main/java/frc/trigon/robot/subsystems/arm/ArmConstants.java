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
import static frc.trigon.robot.utilities.Conversions.offsetRead;

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
            ANGLE_VOLTAGE_COMPANSATION_SATURATION = 12,
            ELEVATOR_VOLTAGE_COMPANSATION_SATURATION = 12;
    private static final boolean
            ANGLE_INVERTED = false,
            ELEVATOR_INVERTED = false;
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
            P = 1,
            I = 0,
            D = 0;
    static final PIDController PID_CONTROLLER = new PIDController(P, I, D);

    static {
        configureAngleEncoder();
        configureElevatorEncoder();
        configureAngleMotor(MASTER_ANGLE_MOTOR);
        configureAngleMotor(FOLLOWER_ANGLE_MOTOR);
        configureElevatorMotor(MASTER_ELEVATOR_MOTOR);
        configureElevatorMotor(FOLLOWER_ELEVATOR_MOTOR);
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

    private static void configureElevatorMotor(CANSparkMax motor) {
        motor.restoreFactoryDefaults();
        motor.setIdleMode(ELEVATOR_IDLE_MODE);
        motor.setInverted(ELEVATOR_INVERTED);
        motor.enableVoltageCompensation(ELEVATOR_VOLTAGE_COMPANSATION_SATURATION);
    }

    private static void configureAngleMotor(CANSparkMax motor) {
        motor.restoreFactoryDefaults();
        motor.setIdleMode(ANGLE_IDLE_MODE);
        motor.setInverted(ANGLE_INVERTED);
        motor.enableVoltageCompensation(ANGLE_VOLTAGE_COMPANSATION_SATURATION);
    }

    private static void configureAngleEncoder() {
        CANcoderConfiguration angleEncoderConfig = new CANcoderConfiguration();
        angleEncoderConfig.MagnetSensor.MagnetOffset = ANGLE_ENCODER_OFFSET;
        angleEncoderConfig.MagnetSensor.AbsoluteSensorRange = ANGLE_ENCODER_RANGE;
        angleEncoderConfig.MagnetSensor.SensorDirection = ANGLE_ENCODER_DIRECTION;
        ANGLE_ENCODER.getConfigurator().apply(angleEncoderConfig);
    }

    private static void configureElevatorEncoder() {
        ELEVATOR_ENCODER.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        ELEVATOR_ENCODER.configFactoryDefault();
        ELEVATOR_ENCODER.setSensorPhase(ELEVATOR_ENCODER_PHASE);
        ELEVATOR_ENCODER.setSelectedSensorPosition(offsetRead(ELEVATOR_ENCODER.getSelectedSensorPosition(), ELEVATOR_ENCODER_OFFSET));
    }
}
