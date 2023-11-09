package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;

public class ArmConstants {
    private static final int
            ANGLE_MASTER_MOTOR_ID = 1,
            ELEVATOR_MASTER_MOTOR_ID = 2,
            ANGLE_FOLLOWER_MOTOR_ID = 3,
            ELEVATOR_FOLLOWER_MOTOR_ID = 4,
            TALON_SRX_ID = 5,
            ANGLE_ENCODER_ID = 6;

    private static final CANSparkMax
            ANGLE_MASTER_MOTOR = new CANSparkMax(ANGLE_MASTER_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
            ELEVATOR_MASTER_MOTOR = new CANSparkMax(ELEVATOR_MASTER_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
            ANGLE_FOLLOWER_MOTOR = new CANSparkMax(ANGLE_FOLLOWER_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
            ELEVATOR_FOLLOWER_MOTOR = new CANSparkMax(ELEVATOR_FOLLOWER_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private static final CANSparkMax.IdleMode
            ELEVATOR_MASTER_IDLE_MODE = CANSparkMax.IdleMode.kCoast,
            ANGLE_MASTER_IDLE_MODE = CANSparkMax.IdleMode.kCoast,
            ELEVATOR_FOLLOWER_IDLE_MODE = CANSparkMax.IdleMode.kCoast,
            ANGLE_FOLLOWER_IDLE_MODE = CANSparkMax.IdleMode.kCoast;
    private static final TalonSRX TALON_SRX = new TalonSRX(TALON_SRX_ID);
    private static final CANcoder ANGLE_ENCODER = new CANcoder(ANGLE_ENCODER_ID);
    private static final NeutralMode TALON_SRX_NEUTRAL_MODE_VALUE = NeutralMode.Coast;
    private static final boolean TALON_SRX_INVERTED = false;
    private static final int
            ANGLE_MASTER_MOTOR_VOLTAGE_COMPENSATION_SATURATION = 12,
            ELEVATOR_MASTER_MOTOR_VOLTAGE_COMPENSATION_SATURATION = 12,
            ANGLE_FOLLOWER_MOTOR_VOLTAGE_COMPENSATION_SATURATION = 12,
            ELEVATOR_FOLLOWER_MOTOR_VOLTAGE_COMPENSATION_SATURATION = 12,
            TALON_SRX_VOLTAGE_COMPENSATION_SATURATION = 12;
    private static final boolean
            ELEVATOR_MASTER_INVERTED = false,
            ANGLE_MASTER_INVERTED = false,
            ELEVATOR_FOLLOWER_INVERTED = false,
            ANGLE_FOLLOWER_INVERTED = false,
            ANGLE_ENCODER_DIRECTION = false;
    private static final AbsoluteSensorRange ANGLE_ENCODER_RANGE = AbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    private static final double ANGLE_ENCODER_OFFSET = 10;
    static final double
            P = 1,
            I = 0,
            D = 0;

    static final PIDController PID_CONTROLLER = new PIDController(P, I, D);

    static {
        configureAngleMasterMotor();
        configureElevatorMasterMotor();
        configureAngleFollowerMotor();
        configureElevatorFollowerMotor();

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

    private static void configureElevatorMasterMotor(){
        ELEVATOR_MASTER_MOTOR.restoreFactoryDefaults();
        ELEVATOR_MASTER_MOTOR.setIdleMode(ELEVATOR_MASTER_IDLE_MODE);
        ELEVATOR_MASTER_MOTOR.setInverted(ELEVATOR_MASTER_INVERTED);
        ELEVATOR_MASTER_MOTOR.enableVoltageCompensation(ELEVATOR_MASTER_MOTOR_VOLTAGE_COMPENSATION_SATURATION);
    }

    private static void configureAngleMasterMotor(){
        ANGLE_MASTER_MOTOR.restoreFactoryDefaults();
        ANGLE_MASTER_MOTOR.setIdleMode(ANGLE_MASTER_IDLE_MODE);
        ANGLE_MASTER_MOTOR.setInverted(ANGLE_MASTER_INVERTED);
        ANGLE_MASTER_MOTOR.enableVoltageCompensation(ANGLE_MASTER_MOTOR_VOLTAGE_COMPENSATION_SATURATION);
    }

    private static void configureElevatorFollowerMotor(){
        ELEVATOR_FOLLOWER_MOTOR.restoreFactoryDefaults();
        ELEVATOR_FOLLOWER_MOTOR.setIdleMode(ELEVATOR_FOLLOWER_IDLE_MODE);
        ELEVATOR_FOLLOWER_MOTOR.setInverted(ELEVATOR_FOLLOWER_INVERTED);
        ELEVATOR_FOLLOWER_MOTOR.enableVoltageCompensation(ELEVATOR_FOLLOWER_MOTOR_VOLTAGE_COMPENSATION_SATURATION);
    }

    private static void configureAngleFollowerMotor(){
        ANGLE_FOLLOWER_MOTOR.restoreFactoryDefaults();
        ANGLE_FOLLOWER_MOTOR.setIdleMode(ANGLE_FOLLOWER_IDLE_MODE);
        ANGLE_FOLLOWER_MOTOR.setInverted(ANGLE_FOLLOWER_INVERTED);
        ANGLE_FOLLOWER_MOTOR.enableVoltageCompensation(ANGLE_FOLLOWER_MOTOR_VOLTAGE_COMPENSATION_SATURATION);
    }

    private static void configureAngleEncoder(){
        CANcoderConfigurator angelEncoderConfig = ANGLE_ENCODER.getConfigurator();


    }
}
