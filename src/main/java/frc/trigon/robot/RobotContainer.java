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
    private final Arm arm = Arm.getInstance();
    private final Roller roller = Roller.getInstance();
    private final Turret turret = Turret.getInstance();
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
        controller.b().whileTrue(ArmCommands.getSetTargetArmStateCommand(ArmConstants.ArmState.TAKE_GROUND_CONE));
        controller.x().whileTrue(ArmCommands.getSetTargetArmStateCommand(ArmConstants.ArmState.PLACE_HIGH_CONE));
        controller.y().whileTrue(ArmCommands.getSetTargetArmStateCommand(ArmConstants.ArmState.PLACE_LOW_CONE));
        controller.rightBumper().whileTrue(ArmCommands.getSetTargetArmStateCommand(ArmConstants.ArmState.PLACE_MEDIUM_CONE));
//        controller.a().whileTrue(ArmCommands.getSetTargetArmStateCommand(ArmConstants.ArmState.TAKE_HIGH_CONE));

        roller.setDefaultCommand(RollerCommands.getFullStopCommand());
        controller.a().whileTrue(RollerCommands.getFullOpeningCommand());

        turret.setDefaultCommand(TurretCommands.getSetMotorPowerFromPositionCommand(new Pose2d(0, 0, Rotation2d.fromDegrees(0))));
    }
}
