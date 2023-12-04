package frc.trigon.robot.subsystems.roller.toohardroller;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;

public class ToohardRollerConstants {
    private static final int
            ANGLE_MOTOR_ID = 0,
            COLLECTION_MOTOR_ID = 1,
            FORWARD_LIMIT_SWITCH_CHANNEL = 0,
            BACKWARD_LIMIT_SWITCH_CHANNEL = 1;

    private static final boolean
            ANGLE_MOTOR_INVERTED = false,
            COLLECTION_MOTOR_INVERTED = false;
    private static final NeutralMode ANGLE_MOTOR_NEUTRAL_MODE = NeutralMode.Brake;
    private static final CANSparkMax.IdleMode COLLECTION_MOTOR_IDLE_MODE = CANSparkMax.IdleMode.kCoast;
    private static final int VOLTAGE_COMPENSATION_SATURATION = 12;
    static final WPI_TalonSRX ANGLE_MOTOR = new WPI_TalonSRX(ANGLE_MOTOR_ID);
    static final CANSparkMax COLLECTION_MOTOR = new CANSparkMax(COLLECTION_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

    static final DigitalInput
            FORWARD_LIMIT_SWITCH = new DigitalInput(FORWARD_LIMIT_SWITCH_CHANNEL),
            BACKWARD_LIMIT_SWITCH = new DigitalInput(BACKWARD_LIMIT_SWITCH_CHANNEL);

    static {
        configureAngleMotor();
        configureCollectionMotor();
    }

    private static void configureAngleMotor() {
        ANGLE_MOTOR.configFactoryDefault();
        ANGLE_MOTOR.setInverted(ANGLE_MOTOR_INVERTED);
        ANGLE_MOTOR.setNeutralMode(ANGLE_MOTOR_NEUTRAL_MODE);

        ANGLE_MOTOR.configVoltageCompSaturation(VOLTAGE_COMPENSATION_SATURATION);
        ANGLE_MOTOR.enableVoltageCompensation(true);
    }

    private static void configureCollectionMotor() {
        COLLECTION_MOTOR.restoreFactoryDefaults();
        COLLECTION_MOTOR.setInverted(COLLECTION_MOTOR_INVERTED);
        COLLECTION_MOTOR.setIdleMode(COLLECTION_MOTOR_IDLE_MODE);
        COLLECTION_MOTOR.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);
    }
}