// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableValue;
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
import frc.robot.subsystems.InNOutSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
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
  final InNOutSubsystem m_inNOutSubsystem = SubsystemConstants.useIntake && SubsystemConstants.useOuttake
      ? new InNOutSubsystem()
      : null;
  final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem(m_poseEstimatorSubsystem);
  final DriveSubsystem m_driveSubsystem = SubsystemConstants.useDrive ? new DriveSubsystem(m_poseEstimatorSubsystem)
      : null;
  final ClimberSubsystem m_climberSubsystem = SubsystemConstants.useClimber ? new ClimberSubsystem() : null;
  final ElevatorSubsystem m_elevatorSubsystem = SubsystemConstants.useElevator ? new ElevatorSubsystem() : null;

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

  private void configureNamedCommands() {
    NamedCommands.registerCommand("intake coral", m_inNOutSubsystem.IntakeCoral());
    NamedCommands.registerCommand("outtake coral", m_inNOutSubsystem.OuttakeCoral());
    NamedCommands.registerCommand("set elevator level", m_elevatorSubsystem.SetPosition(1200));
    // TODO: this is just a random number, make it take a position based on the auto
    // somehow
  }

  private void configureBindings() {
    /*
     * PRODUCTION CODE
     */

    if (SubsystemConstants.useIntake) {
      // SETTING STATES
      // Set state to intaking if intake or elevator beam break are broken
      new Trigger(InNOutSubsystem::intakeHasCoral).and(new Trigger(() -> m_inNOutSubsystem.m_state == "empty"))
          .onTrue(new InstantCommand(() -> {
            m_inNOutSubsystem.m_state = "intaking";
          }));

      // Set state to loaded if isLoaded (elevator beam break not broken and outtake
      // beam break is broken)
      // is true and previous state is intaking
      new Trigger(() -> m_inNOutSubsystem.m_state == "intaking")
          .and(InNOutSubsystem::isLoaded)
          .onTrue(new InstantCommand(() -> {
            m_inNOutSubsystem.m_state = "loaded";
          }));

      // Set state to outtaking if outtake button pressed and we are loaded
      ControllerConstants.m_driveJoystick.button(ControllerConstants.outtakeButton)
          .and(new Trigger(() -> m_inNOutSubsystem.m_state == "loaded"))
          .onTrue(new InstantCommand(() -> {
            m_inNOutSubsystem.m_state = "outtaking";
          }));

      // When it becomes empty (no beam breaks are broken)
      new Trigger(InNOutSubsystem::isEmpty)
          .and(() -> m_inNOutSubsystem.m_state == "outtaking")
          .onTrue(new InstantCommand(() -> {
            m_inNOutSubsystem.m_state = "empty";
          }))
          .onTrue(new InstantCommand(m_inNOutSubsystem::stopOuttake));

      // ACTIONS
      // run intake if intake button pressed and state is empty or is intaking
      (ControllerConstants.m_driveJoystick.button(ControllerConstants.intakeButton)
          .and(new Trigger(() -> m_inNOutSubsystem.m_state == "empty")))
          .or(new Trigger(() -> m_inNOutSubsystem.m_state == "intaking"))
          .whileTrue(new StartEndCommand(m_inNOutSubsystem::runIntake, m_inNOutSubsystem::stopIntake))
          .whileTrue(new StartEndCommand(m_inNOutSubsystem::runOuttake, m_inNOutSubsystem::stopOuttake));

      // Shoot if outtaking and stop when done
      new Trigger(() -> m_inNOutSubsystem.m_state == "outtaking")
          .whileTrue(new StartEndCommand(m_inNOutSubsystem::runOuttake, m_inNOutSubsystem::stopOuttake));
    }

    ControllerConstants.m_driveJoystick.button(5).whileTrue(new RunCommand(() -> {
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
    }, m_driveSubsystem));
    ControllerConstants.m_driveJoystick.button(6).and(new Trigger(() -> {
      return m_driveSubsystem.m_sector.isPresent();
    })).whileTrue(new RunCommand(() -> {
      // m_driveSubsystem.driveToPose(new Pose2d(5.116498 + 8.775 + .45, 4.0199 - .18,
      // new Rotation2d(Math.PI)));
      m_driveSubsystem.driveToPose(DriveConstants.rightAlignPoses[m_driveSubsystem.m_sector.get()]);
    }, m_driveSubsystem));

    ControllerConstants.m_driveJoystick.button(1).whileTrue(new RunCommand(() -> {
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
    ControllerConstants.m_opJoystick.povUp().onTrue(new InstantCommand(() -> {
      m_elevatorSubsystem.goToL4();
      m_elevatorSubsystem.m_targetPosition = Constants.ElevatorConstants.L4Position;
    }));
    ControllerConstants.m_opJoystick.povRight().onTrue(new InstantCommand(() -> {
      m_elevatorSubsystem.goToL3();
      m_elevatorSubsystem.m_targetPosition = Constants.ElevatorConstants.L3Position;
    }));
    ControllerConstants.m_opJoystick.povDown().onTrue(new InstantCommand(() -> {
      m_elevatorSubsystem.goToL2();
      m_elevatorSubsystem.m_targetPosition = Constants.ElevatorConstants.L2Position;
    }));
    ControllerConstants.m_opJoystick.povLeft().onTrue(new InstantCommand(() -> {
      m_elevatorSubsystem.goToL1();
      m_elevatorSubsystem.m_targetPosition = Constants.ElevatorConstants.L2Position;
    }));
    ControllerConstants.m_opJoystick.button(9).onTrue(new InstantCommand(() -> m_elevatorSubsystem.goToElevatorDown()));

    ControllerConstants.m_opJoystick.button(4)
        .onTrue(new InstantCommand(() -> m_elevatorSubsystem.L4ButtonIsPressed()));
    ControllerConstants.m_opJoystick.button(2)
        .onTrue(new InstantCommand(() -> m_elevatorSubsystem.L3ButtonIsPressed()));
    ControllerConstants.m_opJoystick.button(1)
        .onTrue(new InstantCommand(() -> m_elevatorSubsystem.L2ButtonIsPressed()));
    ControllerConstants.m_opJoystick.button(3)
        .onTrue(new InstantCommand(() -> m_elevatorSubsystem.L1ButtonIsPressed()));

    if (SubsystemConstants.useClimber && SubsystemConstants.useIntake) {
      ControllerConstants.m_driveJoystick.button(7)
          .onTrue(new IntakeFoldCommand(m_inNOutSubsystem).withTimeout(ClimberConstants.foldRunTime));
    }

    ControllerConstants.m_driveJoystick.button(1).whileTrue(new ClimberClimbCommand(m_climberSubsystem));

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
