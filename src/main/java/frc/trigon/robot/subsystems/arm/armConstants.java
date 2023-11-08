package frc.trigon.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;

public class armConstants {

    private static final int
            ANGLE_CAN_SPARK_ID = 1,
            ANGLE_NEO_ID = 2,
            ELEVATOR_CAN_SPARK_ID = 3,
            ELEVATOR_NEO_ID = 4;

    private static final CANSparkMax
            CAN_SPARK_ANGLE = new CANSparkMax(ANGLE_CAN_SPARK_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
            CAN_SPARK_ELEVATOR = new CANSparkMax(ELEVATOR_CAN_SPARK_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

    static final double
            P = 1,
            I = 0,
            D = 0;

    static final PIDController PID_CONTROLLER = new PIDController(P, I, D);

    static {
        CAN_SPARK_ANGLE.restoreFactoryDefaults();
        CAN_SPARK_ELEVATOR.restoreFactoryDefaults();
    }

    public enum armState{
        ANGLE,
        ELEVATOR_POSITION
    }
}
