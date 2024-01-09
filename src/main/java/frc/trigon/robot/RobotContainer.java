// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.trigon.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.trigon.robot.subsystems.arm.Arm;
import frc.trigon.robot.subsystems.arm.ArmCommands;
import frc.trigon.robot.subsystems.arm.ArmConstants;
import frc.trigon.robot.subsystems.roller.Roller;
import frc.trigon.robot.subsystems.roller.RollerCommands;
import frc.trigon.robot.subsystems.turret.Turret;
import frc.trigon.robot.subsystems.turret.TurretCommands;

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
