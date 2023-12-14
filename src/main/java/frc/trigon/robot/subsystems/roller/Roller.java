package frc.trigon.robot.subsystems.roller;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Roller extends SubsystemBase {
    private final static Roller INSTANCE = new Roller();
    private final RollerIO rollerIO = RollerIO.generateIO();
    private final RollerInputsAutoLogged rollerInputs = new RollerInputsAutoLogged();

    public static Roller getInstance() {
        return INSTANCE;
    }

    private Roller() {
    }

    @Override
    public void periodic() {
        rollerIO.updateInputs(rollerInputs);
        Logger.processInputs("Roller", rollerInputs);
        updateMechanism();
    }

    void openRoller() {
        rollerIO.setAngleMotorPower(RollerConstants.OPEN_POWER);
    }

    void closeRoller() {
        rollerIO.setAngleMotorPower(RollerConstants.CLOSE_POWER);
    }

    void collect() {
        rollerIO.setCollectionMotorPower(RollerConstants.COLLECTION_MOTOR_SPEED);
    }

    boolean isOpen() {
        return !rollerInputs.forwardLimitSwitchPressed;
    }

    boolean isClosed() {
        return !rollerInputs.backwardLimitSwitchPressed;
    }

    void stopAngleMotor() {
        rollerIO.stopAngleMotor();
    }

    void stopCollectionMotor() {
        rollerIO.stopCollectionMotor();
    }

    private void updateMechanism() {
        if (isOpen())
            RollerConstants.ROLLER_ANGLE_LIGAMENT.setAngle(0);
        else if (isClosed())
            RollerConstants.ROLLER_ANGLE_LIGAMENT.setAngle(90);
        else
            RollerConstants.ROLLER_ANGLE_LIGAMENT.setAngle(45);
        Logger.recordOutput("RollerMechanism", RollerConstants.ROLLER_ANGLE_MECHANISM);
    }
}