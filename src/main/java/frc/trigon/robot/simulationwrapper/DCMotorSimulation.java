package frc.trigon.robot.simulationwrapper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.trigon.robot.subsystems.turret.simulationturret.SimulationTurretConstants;
import frc.trigon.robot.utilities.Conversions;

public class DCMotorSimulation {
    public DCMotorSim motorSim;
    public TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(0, 0);
    private ProfiledPIDController pid;
    private SimpleMotorFeedforward feedforward;

    public void setPID(double p, double i, double d) {
        pid = new ProfiledPIDController(p, i, d, constraints);
    }

    public void setFeedforward(double kS, double kV, double kA) {
        feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    }

    public double calculateTargetMotorVoltage(Rotation2d targetAngle) {
        double pidOutput = pid.calculate(targetAngle.getRotations());
        pid.setGoal(0);
        double feedforward = pid.calculate(pid.getGoal().velocity);
        return pidOutput + feedforward;
    }

    public void setMotorVoltageFromPower(double power) {
        double motorVoltage = MathUtil.clamp(
                Conversions.compensatedPowerToVoltage(power, SimulationTurretConstants.VOLTAGE_COMPENSATION_SATURATION),
                -SimulationTurretConstants.VOLTAGE_COMPENSATION_SATURATION,
                SimulationTurretConstants.VOLTAGE_COMPENSATION_SATURATION
        );
        motorSim.setInputVoltage(motorVoltage);
    }
    /*public static DCMotorSim MOTOR;
    public static TrapezoidProfile.Constraints CONSTRAINTS;
    public static ProfiledPIDController PROFILED_PID_CONTROLLER;
    public static SimpleMotorFeedforward FEEDFORWARD;
    public static double
            MOTION_MAGIC_JERK,
            MOTION_MAGIC_ACCELERATION,
            MOTION_MAGIC_CRUISE_VELOCITY;

    public DCMotorSimulation(DCMotor motorGearbox, double gearRatio, double momentOfInertia) {
        MOTOR = new DCMotorSim(
                motorGearbox,
                gearRatio,
                momentOfInertia
        );
    }

    public void setConstraints(double maxVelocity, double maxAcceleration) {
        CONSTRAINTS = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
    }

    public void setPID(double p, double i, double d) {
        PROFILED_PID_CONTROLLER = new ProfiledPIDController(p, i, d, CONSTRAINTS);
    }

    public void setFeedforward(double kS, double kV, double kA) {
        FEEDFORWARD = new SimpleMotorFeedforward(kS, kV, kA);
    }

    public void setMotionMagicControls(double jerk, double acceleration, double cruiseVelocity) {
        MOTION_MAGIC_JERK = jerk;
        MOTION_MAGIC_ACCELERATION = acceleration;
        MOTION_MAGIC_CRUISE_VELOCITY = cruiseVelocity;
    }*/
}
