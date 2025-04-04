// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.SubsystemConstants;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private PathPlannerAuto m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  public void robotInit() {
    if (SubsystemConstants.useDataManager) {
      DataLogManager.start();
    }
    if (SubsystemConstants.useDrive) {
      addPeriodic(() -> {
        m_robotContainer.m_driveSubsystem.updateOdometry();
      }, 1.0 / DriveConstants.odometryUpdateFrequency);
      m_robotContainer.m_driveSubsystem.startPoseEstimator(new Pose2d(0, 0, new Rotation2d(
          DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red ? Math.PI : 0)));
      FollowPathCommand.warmupCommand().schedule();
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    if (SubsystemConstants.useLimelight) {
      m_robotContainer.m_limelightSubsystem.setThrottle(LimelightConstants.disabledThrottle);
    }

    if (SubsystemConstants.useLimelight) {
      m_robotContainer.m_limelightSubsystem.setLEDs(1);
    }
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    if (SubsystemConstants.useLimelight) {
      m_robotContainer.m_limelightSubsystem.setThrottle(0);
    }
    if (SubsystemConstants.useDrive) {
      m_robotContainer.resetModuleRotationOffsets();
    }

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command
    m_robotContainer.m_driveSubsystem.setYaw(m_autonomousCommand.getStartingPose().getRotation().getDegrees());
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    m_robotContainer.m_inNOutSubsystem.m_state = "empty";

    if (SubsystemConstants.useLimelight) {
      m_robotContainer.m_limelightSubsystem.setLEDs(1);
    }

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    if (SubsystemConstants.useLimelight) {
      m_robotContainer.m_limelightSubsystem.setThrottle(0);
    }
    if (SubsystemConstants.useDrive) {
      m_robotContainer.resetModuleRotationOffsets();
    }
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    if (SubsystemConstants.useLimelight) {
      m_robotContainer.m_limelightSubsystem.setLEDs(1);
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    if (SubsystemConstants.useLimelight) {
      m_robotContainer.m_limelightSubsystem.setThrottle(0);
    }
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
