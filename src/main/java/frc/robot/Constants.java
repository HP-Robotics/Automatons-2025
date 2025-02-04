// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class Constants {

  public static class SubsystemConstants {
    public static final boolean useDrive = false;
    public static final boolean useIntake = true;
    public static final boolean useShooter = false;
    public static final boolean useDataManager = false;
    public static final boolean useLimelight = false;
    public static final boolean usePivot = false;
    public static final boolean useClimber = true;
    public static final boolean useTrigger = false;
  }

  public static class RobotConfigConstants {
    public static final double massKG = 52.16; // TODO: Find out actual value
    public static final double MOI = 0; // TODO: find the actual number
    public static final ModuleConfig moduleConfig = new ModuleConfig(null, null, MOI, null, null, 0);
    public static Translation2d moduleOffsets;
  }

  public static class IDConstants {
    public static final int FLDriveMotorID = 26;
    public static final int FRDriveMotorID = 22;
    public static final int BRDriveMotorID = 20;
    public static final int BLDriveMotorID = 24;

    public static final int FLTurningMotorID = 27;
    public static final int FRTurningMotorID = 23;
    public static final int BRTurningMotorID = 21;
    public static final int BLTurningMotorID = 25;

    public static final int IntakeMotorID = 31;
    public static final int IntakeFoldMotorID = 32;

    public static final int outtakeMotorID = 41;

    public static final int PigeonID = 57;

    public static final int ElevatorMotor1ID = 10; // TODO: Fix the motor ID
    public static final int ElevatorMotor2ID = 11; // TODO: Fix the motor ID

    public static final int ClimbMotorID = 0; // TODO: Find motor ID
  }

  public static class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 5.0; // TODO: Measure with krakens
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = 8.37758; // 480 degrees in radians
    public static final double kMaxAngularAcceleration = 11.1701; // 640 degrees in radians
    public static final double kFastAutoVelocity = 4.5;
    public static final double kfastAutoAcceleration = 3.0;
    public static final double kMaxAutoVelocity = 3;
    public static final double kMaxAutoAcceleration = 3;

    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
  }

  public static class PortConstants {
    public static final int FLAbsEncoder = 12;
    public static final int FRAbsEncoder = 10;
    public static final int BRAbsEncoder = 13;
    public static final int BLAbsEncoder = 11;
  }

  public static class ElevatorConstants {
    public static final double L4Position = 0; // TODO: figure out what these are
    public static final double L3Position = 0; // TODO: figure out what these are
    public static final double ElevatorDownPosition = 0; // TODO: this might be right but should be checked with the
                                                         // other two
    public static final double kP = 0;// TODO: tune these
    public static final double kI = 0;
    public static final double kD = 0;
  }

  public static class ControllerConstants {
    public static final CommandJoystick m_driveJoystick = new CommandJoystick(
        ControllerConstants.kDriverControllerPort);
    public static final CommandJoystick m_opJoystick = new CommandJoystick(ControllerConstants.kOperatorControllerPort);
    public static final boolean useXbox = true;

    public static final int kOperatorControllerPort = 0;
    public static final int kDriverControllerPort = 1;
    public static final double driveJoystickDeadband = useXbox ? 0.15 : 0.15;
    public static final double turnJoystickDeadband = useXbox ? 0.1 : 0.1;

    public static final double driveJoystickExponent = useXbox ? 2 : 2;

    public static final int resetHeadingButton = useXbox ? 8 : 0;
    public static final int resetYawButton = useXbox ? 7 : 11;
    public static final int fieldRelativeButton = useXbox ? 8 : 8;
    public static final int robotRelativeButton = useXbox ? 2 : 8;
    public static final int yuckButton = useXbox ? 4 : 2;
    public static final int climberButton = useXbox ? 10 : 10;
    public static final int intakeButton = useXbox ? 0 : 1;
    public static final int outtakeButton = useXbox ? 0 : 0;
    public static final Trigger outtakeTrigger = m_driveJoystick.button(outtakeButton);
    public static final int drivePointedToSpeakerButton = useXbox ? 6 : 0;
    public static final int drivePointedToNoteButton = useXbox ? 5 : 0;
    public static final int driveToNoteAxis = useXbox ? 2 : 0;
    public static final int driveToAmpButton = 1;
    public static final int pointToCornerButton = 2;

    public static final int L4Button = 9;
    public static final int L3Button = 10;
    public static final int ElevatorDownButton = 11;

    // TODO: Add operator joystick constants

    public static double getRotation(CommandJoystick stick) {
      if (useXbox) {
        return stick.getRawAxis(4);
      } else {
        return stick.getRawAxis(2);
      }
    }
  }

  public static class DriveConstants {
    public static final double kMaxSpeed = 4.4; // meters per second TODO: check this
    public static final double kMaxAngularSpeed = Math.PI * 1.3; // 1/2 rotation per second //auto is 540
    public static final double kModuleMaxAngularSpeed = 10; // TODO: Is right?
    public static final double kSlowSpeed = 2.0;
    public static final double kSlowAngularspeed = Math.PI / 2; // 1/4 rotation per second

    public static final double kWheelRadius = 0.0508 * (218.5 / 225.6); // This is a fudge factor
    public static final double kEncoderResolution = 1.0;

    public static final double driveGearRatio = 6.75 / 1.02; // 1.02 is a fudge factor TODO: move it into the wheel
                                                             // radius
    public static final double turningGearRatio = 540.0 / 35.0;

    public static final Translation2d kFrontLeftLocation = new Translation2d(0.31, 0.305);
    public static final Translation2d kFrontRightLocation = new Translation2d(0.31, -0.305);
    public static final Translation2d kBackLeftLocation = new Translation2d(-0.31, 0.305);
    public static final Translation2d kBackRightLocation = new Translation2d(-0.31, -0.305);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        kFrontLeftLocation, kFrontRightLocation, kBackRightLocation, kBackLeftLocation);
    // TODO: make these the same order as the swerve setpoint generator

    public static final double driveModulekP = 5; // TODO: tune these for 2025
    public static final double driveModulekI = 10;
    public static final double driveModulekD = 0;
    public static final double driveModulekF = 0;

    public static final double turningModulekP = 1.8; // TODO: tune these for 2025
    public static final double turningModulekI = 1;
    public static final double turningModulekD = 0.008;

    public static final double XControllerkP = 6.5;
    public static final double XControllerkI = 0.03;
    public static final double XControllerkD = 1.3;
    public static final double XControllerTolerance = 0.01;
    public static final double XControllerIZone = 0.5;

    public static final double YControllerkP = 6.5;
    public static final double YControllerkI = 0.03;
    public static final double YControllerkD = 1.3;
    public static final double YControllerTolerance = 0.01;
    public static final double YControllerIZone = 0.5;

    public static final double rotationControllerkP = 2;
    public static final double rotationControllerkI = 0.3;// 0.3;
    public static final double rotationControllerkD = 0.15;// 0.15;
    public static final double rotationControllerTolerance = Math.toRadians(2);
    public static final double rotationControllerIZone = 0.15;

    // Absolute encoder values that make the wheels point forward
    // To calculate this, line the wheels up facing forward with the bevel gear
    // pointing to the right(?) or outside(?),
    // and look at the shuffleboard values.
    public static final double absEncoderForwardFL = 0.580;
    public static final double absEncoderForwardFR = 0.280;
    public static final double absEncoderForwardBR = 0.268;
    public static final double absEncoderForwardBL = 0.960;

    public static final double driveCurrentMax = 40.0; // TODO: tune for 2025
    public static final double driveCurrentMin = -40.0;

    public static final double driveRampTimeTo300s = 0.01;
    public static boolean useDriveRamp = false;

    public static final double turnCurrentLimit = 40.0;
    public static final double turningCurrentThreshold = 40.0;
    public static final double turningCurrentTimeThreshold = 0.04;
    public static final double turningRampTimeTo300s = 0.01;

    public static final double odometryUpdateFrequency = 250;
  }

  public static class ClimberConstants {
    public static final double foldSpeed = 0.5;
    public static final double foldRunTime = 2; // TODO: Make this a number that makes sense
  }

  public static class PoseEstimatorConstants {
    // TODO: this is all pasted from last year; PLEASE DON'T USE, FIND REAL VALUES
    public static final Matrix<N3, N1> statesStandardDev = VecBuilder.fill(0.001, 0.001, 0.005);
    public static final double visionXStandardDev = 0.005; // TODO: adjust with framerate
    public static final double visionYStandardDev = 0.005;
    public static final double visionHeadingStandardDev = 0.05;

    public static final double maxAcceptableSkew = Math.PI / 3;
    public static final double maxAcceptableDistance = 5.0;
  }

  public static final class PathplannerPIDConstants {
    public static final PIDConstants translationConstants = new PIDConstants(5, 0, 0);
    public static final PIDConstants rotationConstants = new PIDConstants(5, 0, 0);
  }

  public static class LimelightConstants {
    public static final double inToM = 0.0254;
    public static final Pose2d aprilTagList[] = { // 0 is empty, april tag number is that number in list
        // TODO: update rotations/positions to have a blue origin (only 7 is correct)
        new Pose2d(),
        new Pose2d(7.923198, -3.37068, new Rotation2d(Math.PI * 2 / 3)), // 1
        new Pose2d(7.923198, 3.37068, new Rotation2d(Math.PI * 2 / 3)), // 2
        new Pose2d(2.78681, 4.02961, new Rotation2d(Math.PI)), // 3
        new Pose2d(0.50208, 2.111656, new Rotation2d(Math.PI)), // 4
        new Pose2d(0.50208, -2.111094, new Rotation2d(Math.PI * 3 / 2)), // 5
        new Pose2d(4.700446, -0.719682, new Rotation2d(Math.PI * 3 / 2)), // 6
        new Pose2d(5.116498 + 8.775, 4.0199, new Rotation2d(0)), // 7
        new Pose2d(4.700446, 0.719482, new Rotation2d(0)), // 8
        new Pose2d(3.869358, 0.719482, new Rotation2d(Math.PI / 3)), // 9
        new Pose2d(3.453306, -0.0001, new Rotation2d(Math.PI / 3)), // 10
        new Pose2d(3.869358, -0.719682, new Rotation2d(Math.PI * 5 / 3)), // 11
        new Pose2d(-7.922846, -3.37068, new Rotation2d(Math.PI / 3)), // 12
        new Pose2d(-7.922846, 3.37048, new Rotation2d(Math.PI)), // 13
        new Pose2d(-0.501728, 2.111656, new Rotation2d(0)), // 14
        new Pose2d(-0.501728, -2.111094, new Rotation2d(Math.PI * 2 / 3)), // 15
        new Pose2d(-2.786458, -4.02981, new Rotation2d(Math.PI * 4 / 3)), // 16
        new Pose2d(-4.700094, 0.719682, new Rotation2d(Math.PI * 4 / 3)), // 17
        new Pose2d(-5.1164, -0.00001, new Rotation2d(Math.PI * 4 / 3)), // 18
        new Pose2d(-4.700094, 0.719482, new Rotation2d(Math.PI * 4 / 3)), // 19
        new Pose2d(-3.86926, 0.719482, new Rotation2d(Math.PI * 4 / 3)), // 20
        new Pose2d(-3.452954, -0.0001, new Rotation2d(Math.PI * 4 / 3)), // 21
        new Pose2d(-3.86926, -0.719682, new Rotation2d(Math.PI * 4 / 3)) // 22
    };
  }

  public static class OuttakeConstants {
    public static final double loadSpeed = 0.2;
    public static final double outtakeSpeed = 0.2;
  }
}
