package frc.trigon.robot.subsystems.collector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.utilities.CurrentWatcher;
import org.littletonrobotics.junction.Logger;

public class Collector extends SubsystemBase {
    private final static Collector INSTANCE = new Collector();
    private final CollectorInputsAutoLogged collectorInputs = new CollectorInputsAutoLogged();
    private final CollectorIO collectorIO = CollectorIO.generateIO();

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

    void setPower(double power) {
        collectorIO.setPower(power);
    }

    private void startCurrentWatcher() {
        new CurrentWatcher(
                collectorIO::stop,
                () -> collectorInputs.motorCurrent,
                CollectorConstants.MAX_CURRENT,
                CollectorConstants.MAX_CURRENT_TIME
        );
    }
}