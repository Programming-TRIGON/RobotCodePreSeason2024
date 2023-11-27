package frc.trigon.robot.subsystems.arm.simulationarm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.trigon.robot.subsystems.arm.ArmIO;
import frc.trigon.robot.subsystems.arm.ArmInputsAutoLogged;

public class SimulationArmIO extends ArmIO {
    private final SingleJointedArmSim angleMotor = SimulationArmIOConstants.ANGLE_MOTOR;
    private final ElevatorSim elevatorMotor = SimulationArmIOConstants.ELEVATOR_MOTOR;
    @Override
    protected void updateInputs(ArmInputsAutoLogged inputs) {
    }

    @Override
    protected void stopAngleMotors() {
    }

    @Override
    protected void stopElevatorMotors() {
    }

    @Override
    protected void setTargetAngleState(TrapezoidProfile.State targetState) {
    }

    @Override
    protected void setTargetElevatorState(TrapezoidProfile.State targetState) {
    }
}
