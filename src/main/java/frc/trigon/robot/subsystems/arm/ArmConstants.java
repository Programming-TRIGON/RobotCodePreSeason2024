package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ArmConstants {

    private static final double VOLTAGE_COMPENSATION_SATURATION = 12;

    private static final int
            MASTER_ANGLE_MOTOR_ID = 0,
            FOLLOWER_ANGLE_MOTOR_ID = 1,
            MASTER_ELEVATOR_MOTOR_ID = 2,
            FOLLOWER_ELEVATOR_MOTOR_ID = 3,
            ANGLE_ENCODER_ID = 4,
            ELEVATOR_TALON_SRX_ENCODER_ID = 5;
    static final CANSparkMax
            MASTER_ANGLE_MOTOR = new CANSparkMax(MASTER_ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
            FOLLOWER_ANGLE_MOTOR = new CANSparkMax(FOLLOWER_ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
            MASTER_ELEVATOR_MOTOR = new CANSparkMax(MASTER_ELEVATOR_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
            FOLLOWER_ELEVATOR_MOTOR = new CANSparkMax(FOLLOWER_ELEVATOR_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private static final CANSparkMax.IdleMode
            MASTER_ANGLE_MOTOR_IDLE_MODE = CANSparkMax.IdleMode.kBrake,
            FOLLOWER_ANGLE_MOTOR_IDLE_MODE = CANSparkMax.IdleMode.kBrake,
            MASTER_ELEVATOR_MOTOR_IDLE_MODE = CANSparkMax.IdleMode.kBrake,
            FOLLOWER_ELEVATOR_MOTOR_IDLE_MODE = CANSparkMax.IdleMode.kBrake;
    private static final boolean
            MASTER_ANGLE_MOTOR_INVERTED = false,
            FOLLOWER_ANGLE_MOTOR_INVERTED = false,
            MASTER_ELEVATOR_MOTOR_INVERTED = false,
            FOLLOWER_ELEVATOR_MOTOR_INVERTED = true;

    private static final double ANGLE_ENCODER_OFFSET = 0;
    private static final SensorDirectionValue ANGLE_ENCODER_DIRECTION = SensorDirectionValue.Clockwise_Positive;
    private static final AbsoluteSensorRangeValue ANGLE_ENCODER_RANGE = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    static final CANcoder ANGLE_ENCODER = new CANcoder(ANGLE_ENCODER_ID);
    static final TalonSRX ELEVATOR_TALON_SRX_ENCODER = new TalonSRX(ELEVATOR_TALON_SRX_ENCODER_ID);
    private static final double MAG_ENCODER_OFF_SET = 0;

    private static final double
            ANGLE_P = 1,
            ANGLE_I = 0,
            ANGLE_D = 0,
            ELEVATOR_P = 1,
            ELEVATOR_I = 0,
            ELEVATOR_D = 0;
    static final PIDController
            ANGLE_PID_CONTROLLER = new PIDController(ANGLE_P, ANGLE_I, ANGLE_D),
            ELEVATOR_PID_CONTROLLER = new PIDController(ELEVATOR_P, ELEVATOR_I, ELEVATOR_D);

    private static final double
            MAX_ANGLE_VELOCITY = 600,
            MAX_ANGLE_ACCELERATION = 500;
    static final TrapezoidProfile.Constraints ANGLE_CONSTRAINTS = new TrapezoidProfile.Constraints(MAX_ANGLE_VELOCITY,MAX_ANGLE_ACCELERATION);
    static final StatusSignal<Double> ANGEL_ENCODER_POSITION_SIGNAL = ANGLE_ENCODER.getPosition();
    static final StatusSignal<Double> ANGLE_ENCODER_VELOCITY_SIGNAL = ANGLE_ENCODER.getVelocity();

    private static final double
            ANGLE_MOTOR_KS = 0.5990,
            ANGLE_MOTOR_KV = 0.5990,
            ANGLE_MOTOR_KA = 0.5990,
            ANGLE_MOTOR_KG = 0.5990;
    static final ArmFeedforward ANGLE_MOTOR_FEEDFORWARD = new ArmFeedforward(ANGLE_MOTOR_KS, ANGLE_MOTOR_KG, ANGLE_MOTOR_KV, ANGLE_MOTOR_KA );

    private static void configureMasterAngleMotor() {
        MASTER_ANGLE_MOTOR.restoreFactoryDefaults();
        MASTER_ANGLE_MOTOR.setIdleMode(MASTER_ANGLE_MOTOR_IDLE_MODE);
        MASTER_ANGLE_MOTOR.setInverted(MASTER_ANGLE_MOTOR_INVERTED);
        MASTER_ANGLE_MOTOR.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);
    }

    private static void configureFollowerAngleMotor() {
        FOLLOWER_ANGLE_MOTOR.restoreFactoryDefaults();
        FOLLOWER_ANGLE_MOTOR.setIdleMode(FOLLOWER_ANGLE_MOTOR_IDLE_MODE);
        FOLLOWER_ANGLE_MOTOR.setInverted(FOLLOWER_ANGLE_MOTOR_INVERTED);
        FOLLOWER_ANGLE_MOTOR.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);
    }

    private static void configureMasterElevatorMotor() {
        MASTER_ELEVATOR_MOTOR.restoreFactoryDefaults();
        MASTER_ELEVATOR_MOTOR.setIdleMode(MASTER_ELEVATOR_MOTOR_IDLE_MODE);
        MASTER_ELEVATOR_MOTOR.setInverted(MASTER_ELEVATOR_MOTOR_INVERTED);
        MASTER_ELEVATOR_MOTOR.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);
    }

    private static void configureFollowerElevatorMotor() {
        FOLLOWER_ELEVATOR_MOTOR.restoreFactoryDefaults();
        FOLLOWER_ELEVATOR_MOTOR.setIdleMode(FOLLOWER_ELEVATOR_MOTOR_IDLE_MODE);
        FOLLOWER_ELEVATOR_MOTOR.setInverted(FOLLOWER_ELEVATOR_MOTOR_INVERTED);
        FOLLOWER_ELEVATOR_MOTOR.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);
    }

    private static void configureAngleCanCoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = ANGLE_ENCODER_OFFSET;
        config.MagnetSensor.SensorDirection = ANGLE_ENCODER_DIRECTION;
        config.MagnetSensor.AbsoluteSensorRange = ANGLE_ENCODER_RANGE;
        ANGLE_ENCODER.getConfigurator().apply(config);

        ANGEL_ENCODER_POSITION_SIGNAL.setUpdateFrequency(100);
        ANGLE_ENCODER_VELOCITY_SIGNAL.setUpdateFrequency(100);
        ANGLE_ENCODER.optimizeBusUtilization();
    }

    private static void configureElevatorMagEncoder() {
        ELEVATOR_TALON_SRX_ENCODER.configFactoryDefault();
        ELEVATOR_TALON_SRX_ENCODER.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        ELEVATOR_TALON_SRX_ENCODER.setSensorPhase(false);
        ELEVATOR_TALON_SRX_ENCODER.setSelectedSensorPosition(MAG_ENCODER_OFF_SET, 0, 10);
    }

    static {
        configureFollowerAngleMotor();
        configureFollowerElevatorMotor();
        configureMasterAngleMotor();
        configureMasterElevatorMotor();
        configureAngleCanCoder();
        configureElevatorMagEncoder();
    }

    public enum ArmState {
        COLLECTION(Rotation2d.fromDegrees(-15), 0),
        MIDDLE(Rotation2d.fromDegrees(30), 30),
        HIGH(Rotation2d.fromDegrees(60), 45);

        final Rotation2d angle;
        final double elevatorPosition;

        ArmState(Rotation2d angle, double elevatorPosition) {
            this.angle = angle;
            this.elevatorPosition = elevatorPosition;
        }
    }
}
