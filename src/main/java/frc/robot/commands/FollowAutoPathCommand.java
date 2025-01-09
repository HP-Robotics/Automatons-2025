package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class FollowAutoPathCommand {
    
public FollowAutoPathCommand(DriveSubsystem driveSubsystem, String pathName, Boolean resetPosition) {
    m_driveSubsystem = driveSubsystem;
    m_pathName = pathName;
    m_resetPosition = resetPosition;

    m_path = PathPlannerPath.fromPathFile(pathName);
    m_pathPlannerCommand = PathCommand();
    addRequirements(driveSubsystem);
  }
}