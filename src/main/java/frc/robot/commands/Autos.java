// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.InNOutSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
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

  public static Command DriveForward(DriveSubsystem m_driveSubsystem) {
    return m_driveSubsystem.followPathCommand("Drive Forward From Upper Blue");
  }

  public static Command ThreeCoralBottom(DriveSubsystem m_driveSubsystem) {
    return new PathPlannerAuto("3CoralBottomc3F3");
    // return new SequentialCommandGroup(m_driveSubsystem.followPathCommand("Bottom
    // Cage to C3"),
    // m_driveSubsystem.followPathCommand("C3 to Lower Feeder"),
    // m_driveSubsystem.followPathCommand("Lower Feeder to F3"),
    // m_driveSubsystem.followPathCommand("F3 to Lower Feeder"),
    // m_driveSubsystem.followPathCommand("Lower Feeder to F3"),
    // m_driveSubsystem.followPathCommand("F3 to Lower Feeder"));
  }

}