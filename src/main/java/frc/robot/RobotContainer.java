// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled
 * in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure
 * of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  SendableChooser<Command> autoChooser;
  final PoseEstimatorSubsystem m_poseEstimatorSubsystem = new PoseEstimatorSubsystem();
  final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem(m_poseEstimatorSubsystem);
  final DriveSubsystem m_driveSubsystem = SubsystemConstants.useDrive ? new DriveSubsystem(m_poseEstimatorSubsystem)
      : null;

  final CommandJoystick m_driveJoystick = new CommandJoystick(ControllerConstants.kDriverControllerPort);
  final CommandJoystick m_opJoystick = new CommandJoystick(ControllerConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    if (SubsystemConstants.useDrive) {
      m_driveSubsystem.setDefaultCommand(
          new RunCommand(
              () -> {
                m_driveSubsystem.driveWithJoystick(m_driveJoystick);
              },
              m_driveSubsystem));
    }
    // Configure the trigger bindings
    configureBindings();

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor
   * with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  public void resetDriveOffsets() {
    // The encoders aren't ready immediately
    if (SubsystemConstants.useDrive) {
      m_driveSubsystem.resetOffsets();
    }
  } // TODO: this is a choice (make this not run resetOffsets through three classes)

  private void configureBindings() {
    m_driveJoystick.button(1).and(new Trigger(() -> {
      return m_intakeSubsystem.m_state == "empty";
    })).whileTrue(new InstantCommand(m_intakeSubsystem::startIntake));// Intake
    m_driveJoystick.button(2).and(new Trigger(() -> {
      return m_intakeSubsystem.m_state == "intaking";
    })).whileTrue(new InstantCommand(m_intakeSubsystem::stopIntake));// StopIntake
    m_driveJoystick.button(3).and(new Trigger(() -> {
      return m_intakeSubsystem.m_state == "shoot";
    })).and(m_driveJoystick.button(4)).whileTrue(new InstantCommand(m_intakeSubsystem::shoot));// Shoot
    m_driveJoystick.button(4).and(new Trigger(() -> {
      return m_intakeSubsystem.m_state == "intaking";
    })).whileTrue(new InstantCommand(m_intakeSubsystem::stopIntake));
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

      m_driveJoystick.button(5).whileTrue(new SequentialCommandGroup(new InstantCommand(() -> {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
          m_driveSubsystem.resetPose(FlippingUtil.flipFieldPose(path.getStartingHolonomicPose().get()));
        } else {
          m_driveSubsystem.resetPose(path.getStartingHolonomicPose().get());
        }
        
      }),
          (AutoBuilder.followPath(path))));

    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
    }
    m_driveJoystick.button(6).whileTrue(new RunCommand(
      () -> {
        m_driveSubsystem.driveToPose(new Pose2d());
      }, 
      m_driveSubsystem));

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    // .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
