// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.trigon.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.trigon.robot.constants.RobotConstants;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;


public class Robot extends LoggedRobot {
    public static final boolean IS_REAL = Robot.isReal();
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        configLogger();
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null)
            autonomousCommand.schedule();
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null)
            autonomousCommand.cancel();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    private void configLogger() {
        if (RobotConstants.IS_REPLAY) {
            setUseTiming(false);
            final String logPath = LogFileUtil.findReplayLog();
            final String logWriterPath = LogFileUtil.addPathSuffix(logPath, "_replay");

            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(logWriterPath));
            Logger.start();

            return;
        }

        Logger.addDataReceiver(new WPILOGWriter(RobotConstants.ROBOT_TYPE.loggingPath));
        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();
    }
}
