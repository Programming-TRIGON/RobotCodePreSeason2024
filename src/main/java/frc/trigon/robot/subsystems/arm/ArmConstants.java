package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;

public class ArmConstants {
    // 2 motors for angle - CANSparkMax, Neo (Brushless)
    // 2 motors for elevator/opening - CANSparkMax, Neo (Brushless)
    // CANcoder - angle encoder
    // TalonSRX, mag encoder - elevator encoder
    // PIDController
    // Enum arm state (angle, elevatorPosition)

    // configs:
    // encoders:
    // direction / phase
    // offset
    // (only cancoder) range
    private static final int
            ANGLE_MOTOR_MASTER_ID = 0,
            ANGLE_MOTOR_FOLLOWER_ID = 1,
            ELEVATOR_MOTOR_MASTER_ID = 0,
            ELEVATOR_MOTOR_FOLLOWER_ID = 1,
            ANGLE_ENCODER_ID = 0,
            ELEVATOR_ENCODER_ID = 1;
    private static final boolean
            ANGLE_MOTOR_MASTER_INVERTED = false,
            ANGLE_MOTOR_FOLLOWER_INVERTED = false,
            ELEVATOR_MOTOR_MASTER_INVERTED = false,
            ELEVATOR_MOTOR_FOLLOWER_INVERTED = false;
    private static final CANSparkMax.IdleMode
            ANGLE_MOTORS_IDLE_MODE = CANSparkMax.IdleMode.kBrake,
            ELEVATOR_MOTORS_IDLE_MODE = CANSparkMax.IdleMode.kBrake;
    private static final int VOLTAGE_COMPENSATION_SATURATION = 12;
    private static final int
            P = 0,
            I = 0,
            D = 0;
    private static final CANSparkMax ANGLE_MOTOR_MASTER = new CANSparkMax(ANGLE_MOTOR_MASTER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private static final CANSparkMax ANGLE_MOTOR_FOLLOWER = new CANSparkMax(ANGLE_MOTOR_FOLLOWER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private static final CANSparkMax ELEVATOR_MOTOR_MASTER = new CANSparkMax(ELEVATOR_MOTOR_MASTER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private static final CANSparkMax ELEVATOR_MOTOR_FOLLOWER = new CANSparkMax(ELEVATOR_MOTOR_FOLLOWER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private static final PIDController PID_CONTROLLER = new PIDController(P, I, D);
    private static final int
            ANGLE_ENCODER_DIRECTION = 0,
            ELEVATOR_ENCODER_DIRECTION = 0;
    private static final CANcoder ANGLE_ENCODER = new CANcoder(ANGLE_ENCODER_ID);
    private static final TalonFX ELEVATOR_ENCODER = new TalonFX(ELEVATOR_ENCODER_ID);

    public enum ArmState {
        DEFAULT(Rotation2d.fromDegrees(0), 0),
        TAKE_HIGH_CONE(Rotation2d.fromDegrees(60), 10),
        TAKE_GROUND_CONE(Rotation2d.fromDegrees(0), 0),
        PLACE_LOW_CONE(Rotation2d.fromDegrees(20), 3),
        PLACE_MEDIUM_CONE(Rotation2d.fromDegrees(50), 7),
        PLACE_HIGH_CONE(Rotation2d.fromDegrees(70), 10);

        final Rotation2d angle;
        final double elevatorPosition;

        ArmState(Rotation2d angle, double elevatorPosition) {
            this.angle = angle;
            this.elevatorPosition = elevatorPosition;
        }
    }


    static {
        configureAngleMotors();
        configureElevatorMotors();
        configureAngleEncoder();
        configureElevatorEncoder();
    }

    private static void configureAngleMotors() {
        ANGLE_MOTOR_MASTER.restoreFactoryDefaults();
        ANGLE_MOTOR_MASTER.setInverted(ANGLE_MOTOR_MASTER_INVERTED);
        ANGLE_MOTOR_MASTER.setIdleMode(ANGLE_MOTORS_IDLE_MODE);
        ANGLE_MOTOR_MASTER.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);

        ANGLE_MOTOR_FOLLOWER.restoreFactoryDefaults();
        ANGLE_MOTOR_FOLLOWER.setInverted(ANGLE_MOTOR_FOLLOWER_INVERTED);
        ANGLE_MOTOR_FOLLOWER.setIdleMode(ANGLE_MOTORS_IDLE_MODE);
        ANGLE_MOTOR_FOLLOWER.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);
    }

    private static void configureElevatorMotors() {
        ELEVATOR_MOTOR_MASTER.restoreFactoryDefaults();
        ELEVATOR_MOTOR_MASTER.setInverted(ELEVATOR_MOTOR_MASTER_INVERTED);
        ELEVATOR_MOTOR_MASTER.setIdleMode(ELEVATOR_MOTORS_IDLE_MODE);
        ELEVATOR_MOTOR_MASTER.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);

        ELEVATOR_MOTOR_FOLLOWER.restoreFactoryDefaults();
        ELEVATOR_MOTOR_FOLLOWER.setInverted(ELEVATOR_MOTOR_FOLLOWER_INVERTED);
        ELEVATOR_MOTOR_FOLLOWER.setIdleMode(ELEVATOR_MOTORS_IDLE_MODE);
        ELEVATOR_MOTOR_FOLLOWER.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);
    }

    private static void configureAngleEncoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = config.MagnetSensor.SensorDirection.value;
    }

    private static void configureElevatorEncoder() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Audio.BeepOnConfig = false;
        config.Audio.BeepOnBoot = false;
    }
}
