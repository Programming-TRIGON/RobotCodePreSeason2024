// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.trigon.robot;

import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
    public RobotContainer() {
        configureBindings();
    }

    /**
     * @return the command to run in autonomous mode
     */
    public Command getAutonomousCommand() {
        return null;
    }

    private void configureBindings() {
    }
}
