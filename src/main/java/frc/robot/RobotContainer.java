// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableValue;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.commands.IntakeFoldCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.InNOutSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
  SendableChooser<String> m_autoChooser;
  final PoseEstimatorSubsystem m_poseEstimatorSubsystem = SubsystemConstants.usePoseEstimator
      ? new PoseEstimatorSubsystem()
      : null;
  final InNOutSubsystem m_inNOutSubsystem = SubsystemConstants.useIntake && SubsystemConstants.useOuttake
      ? new InNOutSubsystem()
      : null;
  final LimelightSubsystem m_limelightSubsystem = SubsystemConstants.useLimelight
      ? new LimelightSubsystem(m_poseEstimatorSubsystem)
      : null;
  final DriveSubsystem m_driveSubsystem = SubsystemConstants.useDrive ? new DriveSubsystem(m_poseEstimatorSubsystem)
      : null;
  public final ClimberSubsystem m_climberSubsystem = SubsystemConstants.useClimber ? new ClimberSubsystem() : null;
  final ElevatorSubsystem m_elevatorSubsystem = SubsystemConstants.useElevator ? new ElevatorSubsystem() : null;

  BeamBreak m_intakeBeamBreak = new BeamBreak(0);
  TalonFX m_elevatorMotor1 = new TalonFX(IDConstants.ElevatorMotor1ID);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    configureNamedCommands();
    configureEventTriggers();
    if (SubsystemConstants.useDrive) {
      m_driveSubsystem.configureAutoBuilder();
    }

    if (SubsystemConstants.useDrive) {
      m_driveSubsystem.setDefaultCommand(
          new RunCommand(
              () -> {
                m_driveSubsystem.driveWithJoystick(ControllerConstants.m_driveJoystick);
              },
              m_driveSubsystem));

      // Build an auto chooser. This will use Commands.none() as the default option.
      m_autoChooser = new SendableChooser<String>();
      for (String autoName : AutoBuilder.getAllAutoNames()) {
        m_autoChooser.addOption(autoName, autoName);
      }

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
    NamedCommands.registerCommand("Intake",
        new WaitUntilCommand(m_elevatorSubsystem::atDownPosition).andThen(m_inNOutSubsystem.IntakeCoral()));
    NamedCommands.registerCommand("Outtake", m_inNOutSubsystem.OuttakeCoral().withTimeout(0.5)
        .andThen(new InstantCommand(() -> m_inNOutSubsystem.m_state = "empty")));
    NamedCommands.registerCommand("SetElevatorLevel", m_elevatorSubsystem.SetPosition(1200));
    NamedCommands.registerCommand("GoToL4", m_elevatorSubsystem.GoToL4());
    NamedCommands.registerCommand("GoToL3", m_elevatorSubsystem.GoToL3());
    NamedCommands.registerCommand("GoToL2", m_elevatorSubsystem.GoToL2());
    NamedCommands.registerCommand("GoToL1", m_elevatorSubsystem.GoToL1());
    NamedCommands.registerCommand("ElevatorDown", m_elevatorSubsystem.GoToElevatorDown());
    NamedCommands.registerCommand("WaitForElevator",
        new WaitUntilCommand(m_elevatorSubsystem::atPosition));
    NamedCommands.registerCommand("WaitForIntake",
        new SequentialCommandGroup(
            m_driveSubsystem.StayStillCommand(),
            new WaitUntilCommand(m_inNOutSubsystem::isLoaded)));
    NamedCommands.registerCommand("InitializeElevator", InitializeElevator());
    NamedCommands.registerCommand("Score", new SequentialCommandGroup(
        m_driveSubsystem.StayStillCommand(),
        new WaitUntilCommand(m_elevatorSubsystem::atPosition),
        m_inNOutSubsystem.OuttakeCoral().withTimeout(0.5)
            .andThen(new InstantCommand(() -> m_inNOutSubsystem.m_state = "empty")),
        m_elevatorSubsystem.GoToElevatorDown()));
    // TODO: this is just a random number, make it take a position based on the auto
    // somehow
  }

  public void configureEventTriggers() {
    new EventTrigger("Intake")
        .onTrue(new WaitUntilCommand(m_elevatorSubsystem::atDownPosition).andThen(m_inNOutSubsystem.IntakeCoral()));
    new EventTrigger("Outtake").onTrue(m_inNOutSubsystem.OuttakeCoral().withTimeout(0.5)
        .andThen(new InstantCommand(() -> m_inNOutSubsystem.m_state = "empty")));
    new EventTrigger("SetElevatorLevel").onTrue(m_elevatorSubsystem.SetPosition(1200));
    new EventTrigger("GoToL4").onTrue(m_elevatorSubsystem.GoToL4());
    new EventTrigger("GoToL3").onTrue(m_elevatorSubsystem.GoToL3());
    new EventTrigger("GoToL2").onTrue(m_elevatorSubsystem.GoToL2());
    new EventTrigger("GoToL1").onTrue(m_elevatorSubsystem.GoToL1());
    new EventTrigger("ElevatorDown").onTrue(m_elevatorSubsystem.GoToElevatorDown());
    new EventTrigger("WaitForElevator").onTrue(new WaitUntilCommand(m_elevatorSubsystem::atPosition));
    new EventTrigger("IWaited").onTrue(
        new InstantCommand(() -> SmartDashboard.putString("I waited three", "yes")));
    new EventTrigger("IFished").onTrue(new InstantCommand(() -> SmartDashboard.putString("I fished", "yes")));
    new EventTrigger("WaitForIntake").onTrue(new WaitUntilCommand(m_inNOutSubsystem::isLoaded));
    new EventTrigger("InitializeElevator").onTrue(InitializeElevator());
    new EventTrigger("Score").onTrue(new SequentialCommandGroup(
        new WaitUntilCommand(m_elevatorSubsystem::atPosition),
        m_inNOutSubsystem.OuttakeCoral().withTimeout(0.5)
            .andThen(new InstantCommand(() -> m_inNOutSubsystem.m_state = "empty")),
        m_elevatorSubsystem.GoToElevatorDown()));
  }

  public Command InitializeElevator() {
    if (SubsystemConstants.useElevator && SubsystemConstants.useIntake) {

      return new SequentialCommandGroup(
          new InstantCommand(() -> m_elevatorSubsystem.m_elevatorMotor1.set(-0.2),
              m_elevatorSubsystem),
          new WaitUntilCommand(m_elevatorSubsystem::atBottom),
          new InstantCommand(m_elevatorSubsystem::goToElevatorDown,
              m_elevatorSubsystem),
          new WaitUntilCommand(m_elevatorSubsystem::atDownPosition),
          m_inNOutSubsystem.IntakeCoral(),
          new WaitUntilCommand(m_inNOutSubsystem::isLoaded));
    } else {
      return new WaitCommand(0);
    }
  }

  private void configureBindings() {
    // TEST CODE
    if (SubsystemConstants.useElevator) {
      // ControllerConstants.m_driveJoystick.povUp().whileTrue(
      // new StartEndCommand(m_elevatorSubsystem::GoToTarget, () ->
      // m_elevatorMotor1.setControl(new DutyCycleOut(0)),
      // m_elevatorSubsystem));

      ControllerConstants.m_opJoystick.button(ControllerConstants.elevatorUpButton)
          .whileTrue(new StartEndCommand(() -> {
            m_elevatorMotor1.setControl(new DutyCycleOut(ElevatorConstants.elevatorUpSpeed));
          }, () -> {
            m_elevatorMotor1.setControl(new DutyCycleOut(0));
          }, m_elevatorSubsystem));
      ControllerConstants.m_opJoystick.button(ControllerConstants.elevatorDownButton)
          .whileTrue(new StartEndCommand(() -> {
            m_elevatorMotor1.setControl(new DutyCycleOut(ElevatorConstants.elevatorDownSpeed));
          },
              () -> {
                m_elevatorMotor1.setControl(new DutyCycleOut(0));
              },
              m_elevatorSubsystem));
    }
    /*
     * PRODUCTION CODE
     */

    if (SubsystemConstants.useIntake) {
      // SETTING STATES
      // Set state to intaking if intake or elevator beam break are broken
      ControllerConstants.m_opJoystick.povUp().whileTrue(m_inNOutSubsystem.Dealginate());

      new Trigger(m_inNOutSubsystem::intakeHasCoral).and(new Trigger(() -> m_inNOutSubsystem.m_state == "empty"))
          .onTrue(new InstantCommand(() -> {
            m_inNOutSubsystem.m_state = "intaking"; // TODO: Make sure elevator is at bottom before intaking
          }));
      // TODO: manual override button

      // Set state to loaded if isLoaded (elevator beam break not broken and outtake
      // beam break is broken)
      // is true and previous state is intaking
      new Trigger(() -> m_inNOutSubsystem.m_state == "intaking" || m_inNOutSubsystem.m_state == "empty")
          .and(m_inNOutSubsystem::isLoaded)
          .onTrue(new InstantCommand(() -> {
            m_inNOutSubsystem.m_state = "loaded";
          }));

      // Set state to outtaking if outtake button pressed and we are loaded
      ControllerConstants.outtakeTrigger
          .and(new Trigger(() -> m_inNOutSubsystem.m_state == "loaded"))
          .onTrue(new InstantCommand(() -> {
            m_inNOutSubsystem.m_state = "outtaking";
          }));

      // When it becomes empty (no beam breaks are broken)
      new Trigger(m_inNOutSubsystem::isEmpty)
          .and(() -> m_inNOutSubsystem.m_state == "outtaking")
          .onTrue(new SequentialCommandGroup(
              new InstantCommand(() -> m_inNOutSubsystem.m_state = "empty"),
              new WaitCommand(OuttakeConstants.scoreDelay),
              m_elevatorSubsystem.GoToL1()));

      // ACTIONS
      // run intake if intake button pressed and state is empty or is intaking
      (new Trigger(() -> m_inNOutSubsystem.m_state == "intaking")
          .or(new Trigger(() -> m_inNOutSubsystem.m_state == "empty")))
          .and(ControllerConstants.intakeTrigger)
          .and(new Trigger(m_elevatorSubsystem::atDownPosition))
          .whileTrue(new StartEndCommand(m_inNOutSubsystem::runIntake, m_inNOutSubsystem::stopIntake))
          .whileTrue(new StartEndCommand(m_inNOutSubsystem::loadOuttake, m_inNOutSubsystem::stopOuttake));

      // Shoot if outtaking and stop when done
      new Trigger(() -> m_inNOutSubsystem.m_state == "outtaking")
          .whileTrue(new StartEndCommand(m_inNOutSubsystem::runOuttake, m_inNOutSubsystem::stopOuttake));

      // Manual override button
      ControllerConstants.m_opJoystick.button(ControllerConstants.overrideButton)
          .whileTrue(new InstantCommand(() -> m_inNOutSubsystem.m_state = "override"))
          .whileTrue(new StartEndCommand(m_inNOutSubsystem::runIntake, () -> {
            m_inNOutSubsystem.stopIntake();
            m_inNOutSubsystem.m_state = "empty";
          }))
          .whileTrue(new StartEndCommand(m_inNOutSubsystem::runOuttake, m_inNOutSubsystem::stopOuttake));
    }

    new Trigger(() -> m_inNOutSubsystem.m_state == "loaded")
        .onTrue(new InstantCommand(m_inNOutSubsystem::stopIntake))
        .onTrue(new InstantCommand(m_inNOutSubsystem::stopOuttake));

    if (SubsystemConstants.useDrive) {

      ControllerConstants.m_driveJoystick.button(ControllerConstants.leftAlignButton)
          .and(m_inNOutSubsystem::isEmpty)
          .whileTrue(new RunCommand(() -> {
            // TODO: add auto align to the feeder station if robot doesn't have a coral
            // m_driveSubsystem.driveToPose(new Pose2d(5.116498 + 8.775 + .45, 4.0199 - .18,
            // new Rotation2d(Math.PI)));

            if (m_driveSubsystem.m_feederSector.isPresent()
                && m_driveSubsystem.getDistanceToPose(m_poseEstimatorSubsystem.getPose(),
                    DriveConstants.leftFeederAlignPoses[m_driveSubsystem.m_feederSector
                        .get()]) <= DriveConstants.autoAlignDistanceTolerance
                && (m_driveSubsystem.isNearTargetAngle(m_driveSubsystem.joystickTrans,
                    DriveConstants.feederAlignAngles[m_driveSubsystem.m_feederSector.get()],
                    DriveConstants.autoAlignTolerance)
                    || ControllerConstants.m_driveJoystick
                        .getMagnitude() < ControllerConstants.driveJoystickDeadband)) {
              m_driveSubsystem.driveToPose(DriveConstants.leftFeederAlignPoses[m_driveSubsystem.m_feederSector.get()]);
            } else if (m_driveSubsystem.m_feederSector.isPresent()) {
              m_driveSubsystem.drivePointedTowardsAngle(
                  ControllerConstants.m_driveJoystick,
                  DriveConstants.leftFeederAlignPoses[m_driveSubsystem.m_feederSector.get()].getRotation());
            } else {
              m_driveSubsystem.driveWithJoystick(ControllerConstants.m_driveJoystick);
            }
            m_driveSubsystem.m_driveTrainTable.putValue("Joystick Degrees",
                NetworkTableValue.makeDouble(180 - ControllerConstants.m_driveJoystick.getDirectionDegrees()));
          },
              m_driveSubsystem));

      ControllerConstants.m_driveJoystick.button(ControllerConstants.rightAlignButton)
          .and(m_inNOutSubsystem::isEmpty)
          .whileTrue(new RunCommand(() -> {
            // TODO: add auto align to the feeder station if robot doesn't have a coral
            // m_driveSubsystem.driveToPose(new Pose2d(5.116498 + 8.775 + .45, 4.0199 - .18,
            // new Rotation2d(Math.PI)));

            if (m_driveSubsystem.m_feederSector.isPresent()
                && m_driveSubsystem.getDistanceToPose(m_poseEstimatorSubsystem.getPose(),
                    DriveConstants.rightFeederAlignPoses[m_driveSubsystem.m_feederSector
                        .get()]) <= DriveConstants.autoAlignDistanceTolerance
                && (m_driveSubsystem.isNearTargetAngle(m_driveSubsystem.joystickTrans,
                    DriveConstants.feederAlignAngles[m_driveSubsystem.m_feederSector.get()],
                    DriveConstants.autoAlignTolerance)
                    || ControllerConstants.m_driveJoystick
                        .getMagnitude() < ControllerConstants.driveJoystickDeadband)) {
              m_driveSubsystem.driveToPose(DriveConstants.rightFeederAlignPoses[m_driveSubsystem.m_feederSector.get()]);
            } else if (m_driveSubsystem.m_feederSector.isPresent()) {
              m_driveSubsystem.drivePointedTowardsAngle(
                  ControllerConstants.m_driveJoystick,
                  DriveConstants.rightFeederAlignPoses[m_driveSubsystem.m_feederSector.get()].getRotation());
            } else {
              m_driveSubsystem.driveWithJoystick(ControllerConstants.m_driveJoystick);
            }
            m_driveSubsystem.m_driveTrainTable.putValue("Joystick Degrees",
                NetworkTableValue.makeDouble(180 - ControllerConstants.m_driveJoystick.getDirectionDegrees()));
          },
              m_driveSubsystem));

      ControllerConstants.m_driveJoystick.button(ControllerConstants.rightAlignButton)
          .and(m_inNOutSubsystem::isLoaded)
          .whileTrue(new RunCommand(() -> {
            // TODO: add auto align to the feeder station if robot doesn't have a coral
            // m_driveSubsystem.driveToPose(new Pose2d(5.116498 + 8.775 + .45, 4.0199 - .18,
            // new Rotation2d(Math.PI)));

            if (m_driveSubsystem.m_sector.isPresent()
                && (m_driveSubsystem.isNearTargetAngle(m_driveSubsystem.joystickTrans, m_driveSubsystem.robotToReef,
                    DriveConstants.autoAlignTolerance)
                    || ControllerConstants.m_driveJoystick
                        .getMagnitude() < ControllerConstants.driveJoystickDeadband)) {
              m_driveSubsystem.driveToPose(DriveConstants.rightAlignPoses[m_driveSubsystem.m_sector.get()]);
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

      ControllerConstants.m_driveJoystick.button(ControllerConstants.leftAlignButton)
          .and(m_inNOutSubsystem::isLoaded)
          .whileTrue(new RunCommand(() -> {
            // TODO: add auto align to the feeder station if robot doesn't have a coral
            // m_driveSubsystem.driveToPose(new Pose2d(5.116498 + 8.775 + .45, 4.0199 - .18,
            // new Rotation2d(Math.PI)));

            if (m_driveSubsystem.m_sector.isPresent()
                && (m_driveSubsystem.isNearTargetAngle(m_driveSubsystem.joystickTrans, m_driveSubsystem.robotToReef,
                    DriveConstants.autoAlignTolerance)
                    || ControllerConstants.m_driveJoystick
                        .getMagnitude() < ControllerConstants.driveJoystickDeadband)) {
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

      // ControllerConstants.m_driveJoystick.button(1).whileTrue(new RunCommand(() ->
      // {
      // m_driveSubsystem.drivePointedTowardsAngle(
      // ControllerConstants.m_driveJoystick,
      // Rotation2d.fromDegrees(m_driveSubsystem.getAngleBetweenPoses(m_poseEstimatorSubsystem.getPose(),
      // new Pose2d(
      // DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get()
      // == Alliance.Red
      // ? DriveConstants.redReefCenter
      // : DriveConstants.blueReefCenter,
      // new Rotation2d()))));
      // }, m_driveSubsystem));

      ControllerConstants.resetYawTrigger.onTrue(new InstantCommand(() -> m_driveSubsystem.resetYaw()));
    }

    if (SubsystemConstants.useElevator) {

      // ControllerConstants.m_driveJoystick.button(7)
      // .and(new Trigger(() -> m_inNOutSubsystem.isLoaded()))
      // .whileTrue(new StartEndCommand(m_elevatorSubsystem::GoToTarget,
      // m_elevatorSubsystem::goToElevatorDown));

      ControllerConstants.m_opJoystick.button(ControllerConstants.goToL4Button)
          .and(new Trigger(m_inNOutSubsystem::isLoaded))
          .onTrue(m_elevatorSubsystem.GoToL4());
      ControllerConstants.m_opJoystick.button(ControllerConstants.goToL3Button)
          .and(new Trigger(m_inNOutSubsystem::isLoaded))
          .onTrue(m_elevatorSubsystem.GoToL3());
      ControllerConstants.m_opJoystick.button(ControllerConstants.goToL2Button)
          .and(new Trigger(m_inNOutSubsystem::isLoaded))
          .onTrue((m_elevatorSubsystem.GoToL2()));
      ControllerConstants.m_opJoystick.button(ControllerConstants.goToL1Button)
          // .and(new Trigger(m_inNOutSubsystem::isLoaded))
          .onTrue((m_elevatorSubsystem.GoToL1()));
      ControllerConstants.m_opJoystick.button(ControllerConstants.goToElevatorDownButton)
          .or(ControllerConstants.intakeTrigger)
          .onTrue(m_elevatorSubsystem.GoToElevatorDown());

      // TODO: Make elevator preset and use it after auto aligning
      // ControllerConstants.m_opJoystick.button(14)
      // .onTrue(new InstantCommand(() -> m_elevatorSubsystem.L4ButtonIsPressed()));
      // ControllerConstants.m_opJoystick.button(12)
      // .onTrue(new InstantCommand(() -> m_elevatorSubsystem.L3ButtonIsPressed()));
      // ControllerConstants.m_opJoystick.button(11)
      // .onTrue(new InstantCommand(() -> m_elevatorSubsystem.L2ButtonIsPressed()));
      // ControllerConstants.m_opJoystick.button(13)
      // .onTrue(new InstantCommand(() -> m_elevatorSubsystem.L1ButtonIsPressed()));
    }

    if (SubsystemConstants.useClimber && SubsystemConstants.useIntake) {
      (ControllerConstants.m_driveJoystick.button(ControllerConstants.intakeFoldDualKeyButton))
          .and(ControllerConstants.m_driveJoystick.button(ControllerConstants.intakeFoldButton))
          .onTrue(new IntakeFoldCommand(m_inNOutSubsystem).withTimeout(ClimberConstants.foldRunTime)
              .andThen(new InstantCommand(() -> m_inNOutSubsystem.m_state = "folded")));
    }
    if (SubsystemConstants.useClimber) {
      ControllerConstants.closePinnerTrigger.whileTrue(m_climberSubsystem.closePinner());
      ControllerConstants.climberTrigger.and(new Trigger(() -> m_inNOutSubsystem.m_state == "folded"))
          .onTrue(m_climberSubsystem.Climb())
          .onFalse(m_climberSubsystem.StopClimb());
      ControllerConstants.m_driveJoystick.button(ControllerConstants.intakeFoldDualKeyButton)
          .onTrue(m_climberSubsystem.ResetClimber())
          .onFalse(m_climberSubsystem.StopClimb());
      ControllerConstants.m_driveJoystick.button(ControllerConstants.intakeFoldButton)
          .onTrue(m_climberSubsystem.openPinner());
    }
  }

  /*
   * END PRODUCTION CODE
   */

  /**
   *
   * @return the command to run in autonomous
   */
  public PathPlannerAuto getAutonomousCommand() {
    return (PathPlannerAuto) AutoBuilder.buildAuto(m_autoChooser.getSelected());
  }

}
