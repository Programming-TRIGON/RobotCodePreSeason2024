package frc.trigon.robot.simulationwrapper;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class MotorSimulation {
    public DCMotorSim motorSim;
    public TrapezoidProfile.Constraints constraints;
    public ProfiledPIDController profiledPIDController;
    public PIDController pidController;
    public SimpleMotorFeedforward feedforward;

    public void applyConfigurations(MotorSimulationConfiguration config) {
        if (config.maxVelocity > 0 && config.maxAcceleration > 0)
            this.constraints = new TrapezoidProfile.Constraints(config.maxVelocity, config.maxAcceleration);
        else
            this.constraints = new TrapezoidProfile.Constraints(0, 0);
        this.pidController = new PIDController(config.p, config.i, config.d);
        this.profiledPIDController = new ProfiledPIDController(config.p, config.i, config.d, constraints);
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
