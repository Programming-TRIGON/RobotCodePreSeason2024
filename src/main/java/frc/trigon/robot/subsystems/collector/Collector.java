package frc.trigon.robot.subsystems.collector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.utilities.CurrentWatcher;
import org.littletonrobotics.junction.Logger;

public class Collector extends SubsystemBase {
    private final static Collector INSTANCE = new Collector();
    private final CollectorIO collectorIO = CollectorIO.generateIO();
    private final CollectorInputsAutoLogged collectorInputs = new CollectorInputsAutoLogged();

    public static Collector getInstance() {
        return INSTANCE;
    }

    private Collector() {
        startCurrentWatcher();
    }

    @Override
    public void periodic() {
        collectorIO.updateInputs(collectorInputs);
        Logger.processInputs("Collector", collectorInputs);
    }

    public Command getSetTargetStateCommand(CollectorConstants.CollectorStates state) {
        return new StartEndCommand(
                () -> collectorIO.setPower(state.power),
                () -> {},
                this
        );
    }

    private void startCurrentWatcher(){
        new CurrentWatcher(
                collectorIO::stop,
                () -> collectorInputs.motorCurrent,
                CollectorConstants.MAX_CURRENT,
                CollectorConstants.MAX_CURRENT_TIME
        );
    }
}