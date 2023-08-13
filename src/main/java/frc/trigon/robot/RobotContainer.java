// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.trigon.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.arm.Arm;
import frc.trigon.robot.subsystems.arm.ArmConstants;

public class RobotContainer {
    private final Arm arm = Arm.getInstance();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        arm.setDefaultCommand(arm.getGoToArmStateCommand(ArmConstants.ArmStates.DEFAULT).withName("defaultArmCommand"));
        OperatorConstants.KEYBOARD_CONTROLLER.b().whileTrue(arm.getGoToArmPositionCommand(0.7, Rotation2d.fromDegrees(90)).withName("armOpen"));
        OperatorConstants.KEYBOARD_CONTROLLER.a().whileTrue(arm.getGoToArmPositionCommand(0.7, Rotation2d.fromDegrees(90 + 45)).withName("armClosed"));
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
