// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.InNOutSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class Autos {
  // Example static factory for an autonomous command.
  public static Command exampleAuto(DriveSubsystem subsystem) {
    // return Commands.sequence(subsystem.exampleMethodCommand(), new
    // ExampleCommand(subsystem));
    return null;
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  // public Command DriveForward() {
  // return RunCommand(AutoBuilder.followPath());
  // }
}