// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.commands.ClimberClimbCommand;
import frc.robot.commands.IntakeFoldCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
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
  SendableChooser<Command> m_autoChooser;
  final PoseEstimatorSubsystem m_poseEstimatorSubsystem = SubsystemConstants.usePoseEstimator
      ? new PoseEstimatorSubsystem()
      : null;
  final IntakeSubsystem m_intakeSubsystem = SubsystemConstants.useIntake ? new IntakeSubsystem() : null;
  final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem(m_poseEstimatorSubsystem);
  final DriveSubsystem m_driveSubsystem = SubsystemConstants.useDrive ? new DriveSubsystem(m_poseEstimatorSubsystem)
      : null;
  final ClimberSubsystem m_climberSubsystem = SubsystemConstants.useClimber ? new ClimberSubsystem() : null;
  final ElevatorSubsystem m_elevatorSubsystem = SubsystemConstants.useElevator ? new ElevatorSubsystem() : null;
  final OuttakeSubsystem m_outtakeSubsystem = SubsystemConstants.useOuttake ? new OuttakeSubsystem() : null;

  BeamBreak m_intakeBeamBreak = new BeamBreak(0);
  TalonFX m_elevatorMotor1 = new TalonFX(IDConstants.ElevatorMotor1ID);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    if (SubsystemConstants.useDrive) {
      m_driveSubsystem.setDefaultCommand(
          new RunCommand(
              () -> {
                m_driveSubsystem.driveWithJoystick(ControllerConstants.m_driveJoystick);
              },
              m_driveSubsystem));

      // Build an auto chooser. This will use Commands.none() as the default option.
      m_autoChooser = AutoBuilder.buildAutoChooser();
      // Another option that allows you to specify the default auto by its name
      // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
      SmartDashboard.putData("Auto Chooser", m_autoChooser);
    }
    // Configure the trigger bindings
    configureBindings();
  }

  public void resetModuleRotationOffsets() {
    // The encoders aren't ready immediately so it can't do this in robot init
    if (SubsystemConstants.useDrive) {
      m_driveSubsystem.resetOffsets();
    }
  }

  private void configureBindings() {
    /*
     * TEST CODE
     */
    if (SubsystemConstants.useIntake) {
      ControllerConstants.m_driveJoystick.button(1).and(new Trigger(() -> {
        return m_intakeSubsystem.m_state == "empty";
      })).whileTrue(new InstantCommand(m_intakeSubsystem::doIntaking));// Intake
      ControllerConstants.m_driveJoystick.button(2).and(new Trigger(() -> {
        return m_intakeSubsystem.m_state == "intaking";
      })).whileTrue(new InstantCommand(m_intakeSubsystem::stopIntake));// StopIntake
      ControllerConstants.m_driveJoystick.button(3).and(new Trigger(() -> {
        return m_intakeSubsystem.m_state == "shoot";
      })).and(ControllerConstants.m_driveJoystick.button(4)).whileTrue(new InstantCommand(m_intakeSubsystem::shoot));// Shoot
      ControllerConstants.m_driveJoystick.button(4).and(new Trigger(() -> {
        return m_intakeSubsystem.m_state == "intaking";
      })).whileTrue(new InstantCommand(m_intakeSubsystem::stopIntake));
    }

    // try {
    // PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

    // ControllerConstants.m_driveJoystick.button(5).whileTrue(new
    // SequentialCommandGroup(new InstantCommand(() -> {
    // if (DriverStation.getAlliance().isPresent() &&
    // DriverStation.getAlliance().get() == Alliance.Red) {
    // m_driveSubsystem.resetPose(FlippingUtil.flipFieldPose(path.getStartingHolonomicPose().get()));
    // } else {
    // m_driveSubsystem.resetPose(path.getStartingHolonomicPose().get());
    // }

    // }),
    // (AutoBuilder.followPath(path))));

    // catch (Exception e) {
    // DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
    // }

    /*
     * END TEST CODE
     */

    /*
     * PRODUCTION CODE
     */

    if (SubsystemConstants.useIntake) {
      // SETTING STATES
      // Set state to intaking if intake, elevator, or outtake beam break are broken
      ControllerConstants.m_driveJoystick.button(10)
          .and(new Trigger(IntakeSubsystem::intakeHasCoral))
          .whileTrue(new InstantCommand(() -> {
            m_intakeSubsystem.m_state = "intaking";
          }));

      // Set state to outtaking if outtake button pressed and we are loaded
      ControllerConstants.m_driveJoystick.button(ControllerConstants.outtakeButton)
          .and(new Trigger(IntakeSubsystem::isLoaded))
          .whileTrue(new InstantCommand(() -> {
            m_intakeSubsystem.m_state = "outtaking";
          }));

      // When it becomes empty (no beam breaks are
      // broken)
      new Trigger(IntakeSubsystem::isEmpty)
          .onTrue(new InstantCommand(() -> {
            m_intakeSubsystem.m_state = "empty";
          }))
          .onTrue(new InstantCommand(m_outtakeSubsystem::stopOuttake));

      // ACTIONS
      // run intake if intake button pressed and state is empty; add beam break?
      ControllerConstants.m_driveJoystick.button(ControllerConstants.intakeButton)
          .and(new Trigger(IntakeSubsystem::isLoaded).negate())
          .whileTrue(new InstantCommand(m_intakeSubsystem::doIntaking))
          .whileTrue(new InstantCommand(m_outtakeSubsystem::runOuttake));

      // StopIntake if a button is pressed and state is intaking?
      new Trigger(IntakeSubsystem::isLoaded)
          .onTrue(new InstantCommand(m_intakeSubsystem::stopIntake))
          .onTrue(new InstantCommand(m_outtakeSubsystem::stopOuttake));

      // Shoot if two buttons pressed and state is shoot
      ControllerConstants.m_driveJoystick.button(3).and(new Trigger(IntakeSubsystem::isLoaded)
          .whileTrue(new InstantCommand(m_intakeSubsystem::shoot)));

      // Stop outtake (maybe necessary?) if button is pressed and intake is empty
      ControllerConstants.m_driveJoystick.button(6)
          .and(new Trigger(IntakeSubsystem::isEmpty));
    }

    ControllerConstants.m_driveJoystick.button(5).whileTrue(new RunCommand(
        () -> {
          // m_driveSubsystem.driveToPose(new Pose2d(5.116498 + 8.775 + .45, 4.0199 - .18,
          // new Rotation2d(Math.PI)));

          if (m_driveSubsystem.m_sector.isPresent()
              && (m_driveSubsystem.isNearTargetAngle(m_driveSubsystem.joystickTrans, m_driveSubsystem.robotToReef,
                  Math.PI / 4)
                  || ControllerConstants.m_driveJoystick.getMagnitude() < ControllerConstants.driveJoystickDeadband)) {
            m_driveSubsystem.driveToPose(DriveConstants.leftAlignPoses[m_driveSubsystem.m_sector.get()]);
          } else {
            m_driveSubsystem.drivePointedTowardsAngle(
                ControllerConstants.m_driveJoystick,
                Rotation2d.fromDegrees(m_driveSubsystem.getAngleBetweenPoses(m_poseEstimatorSubsystem.getPose(),
                    new Pose2d(
                        DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
                            ? DriveConstants.redReefCenter
                            : DriveConstants.blueReefCenter,
                        new Rotation2d()))));
          }
          m_driveSubsystem.m_driveTrainTable.putValue("Joystick Degrees",
              NetworkTableValue.makeDouble(180 - ControllerConstants.m_driveJoystick.getDirectionDegrees()));
        },
        m_driveSubsystem));
    ControllerConstants.m_driveJoystick.button(6)
        .and(new Trigger(() -> {
          return m_driveSubsystem.m_sector.isPresent();
        }))
        .whileTrue(new RunCommand(
            () -> {
              // m_driveSubsystem.driveToPose(new Pose2d(5.116498 + 8.775 + .45, 4.0199 - .18,
              // new Rotation2d(Math.PI)));
              m_driveSubsystem.driveToPose(DriveConstants.rightAlignPoses[m_driveSubsystem.m_sector.get()]);
            },
            m_driveSubsystem));

    ControllerConstants.m_driveJoystick.button(1).whileTrue(new RunCommand(
        () -> {
          m_driveSubsystem.drivePointedTowardsAngle(
              ControllerConstants.m_driveJoystick,
              Rotation2d.fromDegrees(m_driveSubsystem.getAngleBetweenPoses(m_poseEstimatorSubsystem.getPose(),
                  new Pose2d(
                      DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
                          ? DriveConstants.redReefCenter
                          : DriveConstants.blueReefCenter,
                      new Rotation2d()))));
        }, m_driveSubsystem));

    if (SubsystemConstants.useDrive) {
      ControllerConstants.m_driveJoystick.button(6).whileTrue(new RunCommand(
          () -> {
            m_driveSubsystem.driveToPose(new Pose2d());
          },
          m_driveSubsystem));
    }
    ControllerConstants.m_opJoystick.povUp().onTrue(
        new InstantCommand(() -> m_elevatorSubsystem.GoToL4()));
    ControllerConstants.m_opJoystick.povRight().onTrue(
        new InstantCommand(() -> m_elevatorSubsystem.GoToL3()));
    ControllerConstants.m_opJoystick.povDown().onTrue(
        new InstantCommand(() -> m_elevatorSubsystem.GoToL2()));
    ControllerConstants.m_opJoystick.povLeft().onTrue(
        new InstantCommand(() -> m_elevatorSubsystem.GoToL1()));
    ControllerConstants.m_opJoystick.button(9).onTrue(
        new InstantCommand(() -> m_elevatorSubsystem.GoToElevatorDown()));

    ControllerConstants.m_opJoystick.button(4).onTrue(
        new InstantCommand(() -> m_elevatorSubsystem.L4ButtonIsPressed()));
    ControllerConstants.m_opJoystick.button(2).onTrue(
        new InstantCommand(() -> m_elevatorSubsystem.L3ButtonIsPressed()));
    ControllerConstants.m_opJoystick.button(1).onTrue(
        new InstantCommand(() -> m_elevatorSubsystem.L2ButtonIsPressed()));
    ControllerConstants.m_opJoystick.button(3).onTrue(
        new InstantCommand(() -> m_elevatorSubsystem.L1ButtonIsPressed()));

    if (SubsystemConstants.useClimber && SubsystemConstants.useIntake) {
      ControllerConstants.m_driveJoystick.button(7)
          .onTrue(new IntakeFoldCommand(m_intakeSubsystem).withTimeout(ClimberConstants.foldRunTime));
    }

    ControllerConstants.m_driveJoystick.button(1)
        .whileTrue(new ClimberClimbCommand(m_climberSubsystem));

    if (SubsystemConstants.useElevator) {
      ControllerConstants.m_driveJoystick.button(ControllerConstants.elevatorUpButton)
          .whileTrue(new StartEndCommand(() -> {
            m_elevatorMotor1.setControl(new DutyCycleOut(ElevatorConstants.elevatorUpSpeed));
          }, () -> {
            m_elevatorMotor1.setControl(new DutyCycleOut(0));
          }, m_elevatorSubsystem));
      ControllerConstants.m_driveJoystick.button(ControllerConstants.elevatorDownButton)
          .whileTrue(new StartEndCommand(() -> {
            m_elevatorMotor1.setControl(new DutyCycleOut(ElevatorConstants.elevatorDownSpeed));
          },
              () -> {
                m_elevatorMotor1.setControl(new DutyCycleOut(0));
              },
              m_elevatorSubsystem));
    }
  }

  /*
   * END PRODUCTION CODE
   */

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
