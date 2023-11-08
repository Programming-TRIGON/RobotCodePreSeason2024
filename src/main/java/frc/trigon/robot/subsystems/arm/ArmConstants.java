package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;

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
            ANGLE_MOTOR_CANSPARKMAX_ID = 0,
            ANGLE_MOTOR_NEO_ID = 1,
            ELEVATOR_MOTOR_CANSPARKMAX_ID = 0,
            ELEVATOR_MOTOR_NEO_ID = 1,
            ANGLE_ENCODER_ID = 0,
            ELEVATOR_ENCODER_ID = 1;
    private static final boolean
            ANGLE_MOTOR_CANSPARKMAX_INVERTED = false,
            ANGLE_MOTOR_NEO_INVERTED = false,
            ELEVATOR_MOTOR_CANSPARKMAX_INVERTED = false,
            ELEVATOR_MOTOR_NEO_INVERTED = false;
    private static final CANSparkMax.IdleMode
            ANGLE_MOTOR_CANSPARKMAX_IDLE_MODE = CANSparkMax.IdleMode.kBrake,
            ANGLE_MOTOR_NEO_IDLE_MODE = CANSparkMax.IdleMode.kBrake,
            ELEVATOR_MOTOR_CANSPARKMAX_IDLE_MODE = CANSparkMax.IdleMode.kBrake,
            ELEVATOR_MOTOR_NEO_IDLE_MODE = CANSparkMax.IdleMode.kBrake;
    private static final int
            ANGLE_MOTOR_CANSPARKMAX_VOLTAGE_COMPENSATION_SATURATION = 12,
            ANGLE_MOTOR_NEO_VOLTAGE_COMPENSATION_SATURATION = 12,
            ELEVATOR_MOTOR_CANSPARKMAX_VOLTAGE_COMPENSATION_SATURATION = 12,
            ELEVATOR_MOTOR_NEO_VOLTAGE_COMPENSATION_SATURATION = 12;
    private static final int
            P = 0,
            I = 0,
            D = 0;
    private static final CANSparkMax ANGLE_MOTOR_CANSPARKMAX = new CANSparkMax(ANGLE_MOTOR_CANSPARKMAX_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private static final CANSparkMax ANGLE_MOTOR_NEO = new CANSparkMax(ANGLE_MOTOR_NEO_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private static final CANSparkMax ELEVATOR_MOTOR_CANSPARKMAX = new CANSparkMax(ELEVATOR_MOTOR_CANSPARKMAX_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private static final CANSparkMax ELEVATOR_MOTOR_NEO = new CANSparkMax(ELEVATOR_MOTOR_NEO_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private static final PIDController PID_CONTROLLER = new PIDController(P, I, D);
    private static final int
            ANGLE_ENCODER_DIRECTION = 0,
            ELEVATOR_ENCODER_PHASE = 0;
    private static final CANcoder ANGLE_ENCODER = new CANcoder(ANGLE_ENCODER_ID);
    private static final TalonSRX ELEVATOR_ENCODER = new TalonSRX(ELEVATOR_ENCODER_ID);

    static {
        configureAngleMotorCANSparkMax();
        configureAngleMotorNeo();
        configureElevatorMotorCANSparkMax();
        configureElevatorMotorNeo();
    }

    private static void configureAngleMotorCANSparkMax() {
        ANGLE_MOTOR_CANSPARKMAX.restoreFactoryDefaults();
        ANGLE_MOTOR_CANSPARKMAX.setInverted(ANGLE_MOTOR_CANSPARKMAX_INVERTED);
        ANGLE_MOTOR_CANSPARKMAX.setIdleMode(ANGLE_MOTOR_CANSPARKMAX_IDLE_MODE);
        ANGLE_MOTOR_CANSPARKMAX.enableVoltageCompensation(ANGLE_MOTOR_CANSPARKMAX_VOLTAGE_COMPENSATION_SATURATION);
    }

    private static void configureAngleMotorNeo() {
        ANGLE_MOTOR_NEO.restoreFactoryDefaults();
        ANGLE_MOTOR_NEO.setInverted(ANGLE_MOTOR_NEO_INVERTED);
        ANGLE_MOTOR_NEO.setIdleMode(ANGLE_MOTOR_NEO_IDLE_MODE);
        ANGLE_MOTOR_NEO.enableVoltageCompensation(ANGLE_MOTOR_NEO_VOLTAGE_COMPENSATION_SATURATION);
    }

    private static void configureElevatorMotorCANSparkMax() {
        ELEVATOR_MOTOR_CANSPARKMAX.restoreFactoryDefaults();
        ELEVATOR_MOTOR_CANSPARKMAX.setInverted(ELEVATOR_MOTOR_CANSPARKMAX_INVERTED);
        ELEVATOR_MOTOR_CANSPARKMAX.setIdleMode(ELEVATOR_MOTOR_CANSPARKMAX_IDLE_MODE);
        ELEVATOR_MOTOR_CANSPARKMAX.enableVoltageCompensation(ELEVATOR_MOTOR_CANSPARKMAX_VOLTAGE_COMPENSATION_SATURATION);
    }

    private static void configureElevatorMotorNeo() {
        ELEVATOR_MOTOR_NEO.restoreFactoryDefaults();
        ELEVATOR_MOTOR_NEO.setInverted(ELEVATOR_MOTOR_NEO_INVERTED);
        ELEVATOR_MOTOR_NEO.setIdleMode(ELEVATOR_MOTOR_NEO_IDLE_MODE);
        ELEVATOR_MOTOR_NEO.enableVoltageCompensation(ELEVATOR_MOTOR_NEO_VOLTAGE_COMPENSATION_SATURATION);
    }
}
