package frc.robot.commands;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class IntakeFoldCommand {
  DriveSubsystem m_driveSubsystem;
  PathFollowingController m_driveController;
  RobotConfig m_config;
  PathPlannerTrajectoryState targetState;
  Pose2d currentPose;

  public IntakeFoldCommand(DriveSubsystem subsystem) {

  }

  public void fold() {
    if (Constants.SubsystemConstants.useIntake == true) {
      new SequentialCommandGroup(

      );
    }

  }
}
