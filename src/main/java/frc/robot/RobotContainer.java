// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.commands.IntakeFoldCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
  final IntakeSubsystem m_intakeSubsystem = SubsystemConstants.useIntake ? new IntakeSubsystem() : null;
  final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem(m_poseEstimatorSubsystem);
  final DriveSubsystem m_driveSubsystem = SubsystemConstants.useDrive ? new DriveSubsystem(m_poseEstimatorSubsystem)
      : null;

  final CommandJoystick m_driveJoystick = new CommandJoystick(ControllerConstants.kDriverControllerPort);
  final CommandJoystick m_opJoystick = new CommandJoystick(ControllerConstants.kOperatorControllerPort);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // This is for testing change this
    m_poseEstimatorSubsystem.createPoseEstimator(DriveConstants.kDriveKinematics,
        new Rotation2d(), new SwerveModulePosition[] {
            new SwerveModulePosition(0, new Rotation2d()),
            new SwerveModulePosition(0, new Rotation2d()),
            new SwerveModulePosition(0, new Rotation2d()),
            new SwerveModulePosition(0, new Rotation2d())
        }, new Pose2d(0, 0, new Rotation2d(Math.PI)));

    if (SubsystemConstants.useDrive) {
      m_driveSubsystem.setDefaultCommand(
          new RunCommand(
              () -> {
                m_driveSubsystem.driveWithJoystick(m_driveJoystick);
              },
              m_driveSubsystem));

      // Build an auto chooser. This will use Commands.none() as the default option.
      autoChooser = AutoBuilder.buildAutoChooser();
      // Another option that allows you to specify the default auto by its name
      // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

      SmartDashboard.putData("Auto Chooser", autoChooser);
    }
    // Configure the trigger bindings
    configureBindings();
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
  private void configureBindings() {
    m_driverController.button(1).and(new Trigger(() -> {
      return m_intakeSubsystem.m_state == "empty";
    })).whileTrue(new InstantCommand(m_intakeSubsystem::startIntake));// Intake
    m_driverController.button(2).and(new Trigger(() -> {
      return m_intakeSubsystem.m_state == "intaking";
    })).whileTrue(new InstantCommand(m_intakeSubsystem::stopIntake));// StopIntake
    m_driverController.button(3).and(new Trigger(() -> {
      return m_intakeSubsystem.m_state == "shoot";
    })).and(m_driverController.button(4)).whileTrue(new InstantCommand(m_intakeSubsystem::shoot));// Shoot
    m_driverController.button(4).and(new Trigger(() -> {
      return m_intakeSubsystem.m_state == "intaking";
    })).whileTrue(new InstantCommand(m_intakeSubsystem::stopIntake));
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    // .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    if (SubsystemConstants.useClimber && SubsystemConstants.useIntake) {
      m_driveJoystick.button(7).onTrue(new IntakeFoldCommand(m_intakeSubsystem));
    }
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
