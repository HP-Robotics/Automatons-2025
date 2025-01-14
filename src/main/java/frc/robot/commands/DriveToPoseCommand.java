// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.PIDConstantsOurs;
import frc.robot.Constants.RobotConfigConstants;
import frc.robot.subsystems.DriveSubsystem;
import com.pathplanner.lib.commands.PathfindThenFollowPath;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPoseCommand extends Command {
  /** Creates a new DriveToPoseCommand. */
  DriveSubsystem m_driveSubsystem;
  String m_pathName;
  PathPlannerPath m_path;
  PathfindingCommand m_PathfindingCommand;
  PathfindThenFollowPath m_pathPlannerCommand;
  PathFollowingController m_driveController;
  RobotConfig m_config;
  PathPlannerTrajectoryState targetState;
  Pose2d currentPose;

  public DriveToPoseCommand(DriveSubsystem driveSubsystem, String pathName) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSubsystem;
    try{
      //m_config = RobotConfig.fromGUISettings();
    m_config = new RobotConfig(RobotConfigConstants.massKG,RobotConfigConstants.MOI,RobotConfigConstants.moduleConfig,RobotConfigConstants.moduleOffsets);
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
    m_pathName = pathName;
    try {
      m_path = PathPlannerPath.fromPathFile(pathName);
    }
    catch(Exception e) {
      System.out.println("Path Exception");
    }
    m_pathPlannerCommand = PathCommand();
    addRequirements(driveSubsystem);
  }
  public PathfindThenFollowPath PathCommand() {
    return new PathfindThenFollowPath(
        m_path,
        new PathConstraints(Constants.AutoConstants.kMaxAutoVelocity,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared,
            Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond,
            Constants.AutoConstants.kMaxAngularAcceleration),
        m_driveSubsystem::getPose,
        m_driveSubsystem::getCurrentspeeds,
        m_driveSubsystem::driveRobotRelative,
        m_driveController = new PPHolonomicDriveController(
  PIDConstantsOurs.translationConstants, PIDConstantsOurs.rotationConstants),
        m_config,
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == Alliance.Red) {
              return true;
            }
          }
          return false;
        },
        m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pathPlannerCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_pathPlannerCommand.execute();
    PathPlannerPath path;
    try {
      path = PathPlannerPath.fromPathFile("UpperReef3Coral");
    } catch (FileVersionException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } catch (ParseException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    PathConstraints constraints = new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));
    Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
        m_path,
        constraints);
    AutoBuilder.pathfindThenFollowPath(m_path, constraints).schedule();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pathPlannerCommand.end(interrupted);
    m_driveSubsystem.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_pathPlannerCommand.isFinished();
  }
}