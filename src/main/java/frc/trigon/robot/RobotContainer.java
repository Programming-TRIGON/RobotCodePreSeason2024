// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.trigon.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.trigon.robot.subsystems.arm.Arm;
import frc.trigon.robot.subsystems.arm.ArmCommands;
import frc.trigon.robot.subsystems.arm.ArmConstants;
import frc.trigon.robot.subsystems.roller.Roller;

public class RobotContainer {
    private final Arm arm = Arm.getInstance();
    private final Roller roller = Roller.getInstance();
    private final CommandXboxController controller = new CommandXboxController(0);

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
        arm.setDefaultCommand(ArmCommands.getSetTargetArmStateCommand(ArmConstants.ArmState.DEFAULT));
//        controller.a().whileTrue();
    }
}
