// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.trigon.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.trigon.robot.subsystems.arm.Arm;
import frc.trigon.robot.subsystems.arm.ArmCommands;
import frc.trigon.robot.subsystems.arm.ArmConstants;
import frc.trigon.robot.subsystems.turret.Turret;
import frc.trigon.robot.subsystems.turret.TurretCommands;

public class RobotContainer {
    private final CommandXboxController controller = new CommandXboxController(0);
    private final Translation2d translation = new Translation2d(4, 9);
    private final Pose2d pose = new Pose2d(translation, Rotation2d.fromDegrees(50));
    private final Arm arm = Arm.getInstance();
    private final Turret turret = Turret.getInstance();

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
        arm.setDefaultCommand(ArmCommands.getSetTargetArmStateCommand(ArmConstants.ArmState.DEFAULT_STATE));
        turret.setDefaultCommand(TurretCommands.getAlignTurretToHubCommand(() -> pose));
        controller.a().whileTrue(ArmCommands.getSetTargetArmStateCommand(ArmConstants.ArmState.CONE_COLLECTION));
        controller.b().whileTrue(ArmCommands.getSetTargetArmStateCommand(ArmConstants.ArmState.HIGH_CONE));
        controller.x().whileTrue(ArmCommands.getSetTargetArmStateCommand(ArmConstants.ArmState.LOW_CONE));
    }
}