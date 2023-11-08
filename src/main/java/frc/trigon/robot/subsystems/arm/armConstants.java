package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;

public class armConstants {

    private static final int
            ANGLE_CAN_SPARK_ID = 1,
            ELEVATOR_CAN_SPARK_ID = 2,
            ANGLE_NEO_ID = 3,
            ELEVATOR_NEO_ID = 4,
            TALON_SRX_ID = 5,
            CAN_CODER_ID = 6;

    private static final CANSparkMax
            CAN_SPARK_ANGLE = new CANSparkMax(ANGLE_CAN_SPARK_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
            CAN_SPARK_ELEVATOR = new CANSparkMax(ELEVATOR_CAN_SPARK_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private static final TalonSRX TALON_SRX = new TalonSRX(TALON_SRX_ID);
    private static final CANCoder CAN_CODER = new CANCoder(CAN_CODER_ID);

    private static final NeutralMode TALON_SRX_NEUTRAL_MODE_VALUE = NeutralMode.Coast;
    private static final boolean TALON_SRX_INVERTED = false;
    private static final int TALON_SRX_VOLTAGE_COMPENSATION_SATURATION = 12;

    private static final boolean CAN_CODER_DIRECTION = false;
    private static final AbsoluteSensorRange CAN_CODER_RANGE = new AbsoluteSensorRange(0.5, -0.5);
    private static final double CAN_CODER_OFFSET = 10;

    static final double
            P = 1,
            I = 0,
            D = 0;

    static final PIDController PID_CONTROLLER = new PIDController(P, I, D);

    static {
        CAN_SPARK_ANGLE.restoreFactoryDefaults();
        CAN_SPARK_ELEVATOR.restoreFactoryDefaults();

        TALON_SRX.configFactoryDefault();
        TALON_SRX.configVoltageCompSaturation(TALON_SRX_VOLTAGE_COMPENSATION_SATURATION);
        TALON_SRX.enableVoltageCompensation(true);
        TALON_SRX.setNeutralMode(TALON_SRX_NEUTRAL_MODE_VALUE);
        TALON_SRX.setInverted(TALON_SRX_INVERTED);

        CAN_CODER.configFactoryDefault();
        CANCoderConfiguration canCoderConfig = new CANCoderConfiguration();
        canCoderConfig.sensorDirection = CAN_CODER_DIRECTION;
        canCoderConfig.absoluteSensorRange = CAN_CODER_RANGE;
        canCoderConfig.magnetOffsetDegrees = CAN_CODER_OFFSET;
        CAN_CODER.configAllSettings(canCoderConfig);
    }

    public enum armState{
        ANGLE,
        ELEVATOR_POSITION
    }
}
