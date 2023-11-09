package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;

public class ArmConstants {
    private static final int
            ANGLE_MOTOR_ID = 1,
            ELEVATOR_MOTOR_ID = 2,
            ANGLE_NEO_ID = 3,
            ELEVATOR_NEO_ID = 4,
            TALON_SRX_ID = 5,
            ANGLE_ENCODER_ID = 6;

    private static final CANSparkMax
            ANGLE_MOTOR = new CANSparkMax(ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
            ELEVATOR_MOTOR = new CANSparkMax(ELEVATOR_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private static final CANSparkMax.IdleMode
            ELEVATOR_IDLE_MODE = CANSparkMax.IdleMode.kCoast,
            ANGLE_IDLE_MODE = CANSparkMax.IdleMode.kCoast;
    private static final TalonSRX TALON_SRX = new TalonSRX(TALON_SRX_ID);
    private static final CANCoder ANGLE_ENCODER = new CANCoder(ANGLE_ENCODER_ID);
    private static final NeutralMode TALON_SRX_NEUTRAL_MODE_VALUE = NeutralMode.Coast;
    private static final boolean TALON_SRX_INVERTED = false;
    private static final int
            ANGLE_MOTOR_VOLTAGE_COMPENSATION_SATURATION = 12,
            ELEVATOR_MOTOR_VOLTAGE_COMPENSATION_SATURATION = 12,
            TALON_SRX_VOLTAGE_COMPENSATION_SATURATION = 12;
    private static final boolean
            ELEVATOR_INVERTED = false,
            ANGLE_INVERTED = false,
            ANGLE_ENCODER_DIRECTION = false;
    private static final AbsoluteSensorRange ANGLE_ENCODER_RANGE = AbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    private static final double ANGLE_ENCODER_OFFSET = 10;
    static final double
            P = 1,
            I = 0,
            D = 0;

    static final PIDController PID_CONTROLLER = new PIDController(P, I, D);

    static {
        configureAngleMotor();
        configureElevatorMotor();

        TALON_SRX.configFactoryDefault();
        TALON_SRX.configVoltageCompSaturation(TALON_SRX_VOLTAGE_COMPENSATION_SATURATION);
        TALON_SRX.enableVoltageCompensation(true);
        TALON_SRX.setNeutralMode(TALON_SRX_NEUTRAL_MODE_VALUE);
        TALON_SRX.setInverted(TALON_SRX_INVERTED);

        configureAngleEncoder();
    }

    public enum ArmState{
        ANGLE(0.5),
        ELEVATOR_POSITION(-0.5);

        final double rotation2d;
        ArmState(double rotation2d){
            this.rotation2d = rotation2d;
        }
    }

    private static void configureElevatorMotor(){
        ELEVATOR_MOTOR.restoreFactoryDefaults();
        ELEVATOR_MOTOR.setIdleMode(ELEVATOR_IDLE_MODE);
        ELEVATOR_MOTOR.setInverted(ELEVATOR_INVERTED);
        ELEVATOR_MOTOR.enableVoltageCompensation(ELEVATOR_MOTOR_VOLTAGE_COMPENSATION_SATURATION);
    }

    private static void configureAngleMotor(){
        ANGLE_MOTOR.restoreFactoryDefaults();
        ANGLE_MOTOR.setIdleMode(ANGLE_IDLE_MODE);
        ANGLE_MOTOR.setInverted(ANGLE_INVERTED);
        ANGLE_MOTOR.enableVoltageCompensation(ANGLE_MOTOR_VOLTAGE_COMPENSATION_SATURATION);
    }

    private static void configureAngleEncoder(){
        ANGLE_ENCODER.configFactoryDefault();
        CANCoderConfiguration angleEncoderConfig = new CANCoderConfiguration();
        angleEncoderConfig.sensorDirection = ANGLE_ENCODER_DIRECTION;
        angleEncoderConfig.absoluteSensorRange = ANGLE_ENCODER_RANGE;
        angleEncoderConfig.magnetOffsetDegrees = ANGLE_ENCODER_OFFSET;
        ANGLE_ENCODER.configAllSettings(angleEncoderConfig);
    }
}
