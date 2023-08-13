// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.trigon.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.trigon.robot.constants.CommandsConstants;
import frc.trigon.robot.subsystems.arm.Arm;
import frc.trigon.robot.subsystems.collector.Collector;
import frc.trigon.robot.subsystems.roller.Roller;
import frc.trigon.robot.subsystems.sideshooter.SideShooter;
import frc.trigon.robot.subsystems.swerve.Swerve;

public class RobotContainer {
    private final Arm arm = Arm.getInstance();
    private final SideShooter sideShooter = SideShooter.getInstance();
    private final Collector collector = Collector.getInstance();
    private final Roller roller = Roller.getInstance();
    private final Swerve swerve = Swerve.getInstance();

    public RobotContainer() {
        bindCommands();
    }

    /**
     * @return the autonomous command
     */
    public Command getAutonomousCommand() {
        return null;
    }

    private void bindCommands() {
        bindDefaultCommands();
        bindControllerCommands();
    }

    private void bindControllerCommands() {
    }

    private void bindDefaultCommands() {
        arm.setDefaultCommand(CommandsConstants.GO_TO_DEFAULT_ARM_STATE_COMMAND);
        sideShooter.setDefaultCommand(CommandsConstants.GO_TO_DEFAULT_SIDE_SHOOTER_STATE_COMMAND);
        collector.setDefaultCommand(CommandsConstants.STOP_COLLECTOR_COMMAND);
        roller.setDefaultCommand(roller.getFullCloseCommand());
        swerve.setDefaultCommand(CommandsConstants.FIELD_RELATIVE_DRIVE_COMMAND);
    }
}
