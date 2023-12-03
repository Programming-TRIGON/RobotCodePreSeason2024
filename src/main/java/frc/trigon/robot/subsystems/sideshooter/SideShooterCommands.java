package frc.trigon.robot.subsystems.sideshooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;

public class SideShooterCommands {
    private static final SideShooter SIDE_SHOOTER = SideShooter.getInstance();
    public static Command getSetTargetAngleCommand(Rotation2d targetAngle){
        return new FunctionalCommand(
                ()-> SideShooter.getInstance().generateAngleMotorProfile(targetAngle),
                SIDE_SHOOTER::setTargetAngleFromProfile,
                (interrupted) -> {
                },
                ()-> false,
                SIDE_SHOOTER
        );
    }

    public static Command getSetVoltageShootingCommand(double Voltage){
        return new StartEndCommand(
                ()-> SideShooter.getInstance().setTargetShootingVoltage(Voltage),
                ()-> SideShooter.getInstance().stopShooting(),
                SIDE_SHOOTER
        );
    }

    public static Command getSetTargetStateCommand(boolean byOrder, SideShooterConstants.SideShooterState state){
        if (byOrder){
            return new ParallelCommandGroup(
                    getSetTargetAngleCommand(state.angle),
                    getSetVoltageShootingCommand(state.voltage)
            );
        }else {
            return new SequentialCommandGroup(
                    getSetTargetAngleCommand(state.angle),
                    getSetVoltageShootingCommand(state.voltage)
            );
        }
    }
}





