// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.trigon.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.trigon.robot.subsystems.arm.Arm;
import frc.trigon.robot.subsystems.arm.ArmCommands;

public class RobotContainer {
    private final CommandXboxController controller = new CommandXboxController(0);
    private final Arm arm = Arm.getInstance();

    public RobotContainer() {
        configureBindings();
    }

    /**
     * @return the command to run in autonomous mode
     */
    public Command getAutonomousCommand() {
        return null;
    }
    private final Rotation2d rotation2d =  Rotation2d.fromDegrees(90);

    private void configureBindings() {
        arm.setDefaultCommand(ArmCommands.getSetTargetArmPositionCommand(rotation2d, 50));
        ArmCommands.getSetTargetArmPositionCommand(rotation2d, 50).schedule();
        controller.a().whileTrue(ArmCommands.getSetTargetArmPositionCommand(rotation2d, 50));

    }
}
