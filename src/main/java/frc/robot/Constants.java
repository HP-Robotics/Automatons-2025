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
import com.pathplanner.lib.util.FlippingUtil;

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
    public static final boolean useOuttake = true;
    public static final boolean useDataManager = false;
    public static final boolean useLimelight = false;
    public static final boolean useClimber = true;
    public static final boolean usePoseEstimator = false;
    public static final boolean useElevator = true;
  }

  public static class RobotConfigConstants {
    public static final double massKG = 52.16; // TODO: Find out actual value
    public static final double MOI = 0; // TODO: find the actual number
    public static final ModuleConfig moduleConfig = new ModuleConfig(null, null, MOI, null, null, 0);
    public static Translation2d moduleOffsets;
  }

  public static class IDConstants {
    public static final int ClimberMotorID = 11;

    public static final int FLDriveMotorID = 26;
    public static final int FRDriveMotorID = 22;
    public static final int BRDriveMotorID = 20;
    public static final int BLDriveMotorID = 24;

    public static final int FLTurningMotorID = 27;
    public static final int FRTurningMotorID = 23;
    public static final int BRTurningMotorID = 21;
    public static final int BLTurningMotorID = 25;

    public static final int intakeMotorID = 31;
    public static final int intakeFoldMotorID = 32;

    public static final int outtakeMotorID = 41;

    public static final int PigeonID = 57;

    public static final int ElevatorMotor1ID = 60; // TODO: Fix the motor ID
    public static final int ElevatorMotor2ID = 61; // TODO: Fix the motor ID

    public static final int climbMotorID = 11; // TODO: Find motor ID
    public static final int pinnerMotorID = 12;
    public static final int releaseMotorID = 13;

    public static final int pinnerAbsEncoderID = 7;
    public static final int dealginatorMotorID = 35;
  }

  public static class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 5.0; // TODO: Measure with krakens
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.6; // 3
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
    public static final int intakeBeamBreakID = 0; // this is fake
    public static final int outtakeBeamBreakID = 9;
    public static final int elevatorBeamBreakID = 8;
  }

  public static class ElevatorConstants {
    public static final double L4Position = 76; // TODO: figure out what these are
    public static final double L3Position = 51.5;
    public static final double L2Position = 35.5;
    public static final double L1Position = 20;
    public static final double elevatorDownPosition = 2.6; // Original: 3.5
    public static final double bottomPosition = 0;
    // TODO: this might be right but should be checked with the other two
    public static final double kP = 1.5;// TODO: tune these more
    public static final double kI = 0.3;
    public static final double kD = 0;

    public static final double errorTolerance = 1.0;

    public static final double upperLimit = 20;

    public static final double elevatorUpSpeed = 1; // 0.15 // 0.6 or 0.7 and -0.4
    public static final double elevatorDownSpeed = -0.1;
    public static final double kG = 0.8;
    public static final double kA = 0.0;
    public static final double kS = 0.0;
    public static final double kV = 0.0;

    public static final double motionMagicCruiseVelocity = 150;
    public static final double motionMagicAcceleration = 120;
    public static final double motionMagicJerk = motionMagicCruiseVelocity * 10;
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

    // public static final int resetYawButton = useXbox ? 7 : 11;
    // public static final int fieldRelativeButton = useXbox ? 8 : 8;
    // public static final int robotRelativeButton = useXbox ? 2 : 8;

    // DRIVER BUTTONS
    public static final Trigger climberTrigger = m_driveJoystick.axisGreaterThan(2, 0.2);
    public static final int intakeButton = 2;
    public static final Trigger outtakeTrigger = m_driveJoystick.button(4);
    public static final int leftAlignButton = 5;
    public static final int rightAlignButton = 6;
    public static final int intakeFoldButton = 7;
    public static final int intakeFoldDualKeyButton = 8;
    public static final Trigger closePinnerButton = m_driveJoystick.axisGreaterThan(3, 0.2);

    // OPERATOR BUTTONS
    public static final Trigger elevatorL3Trigger = m_opJoystick.button(3);
    public static final Trigger elevatorL4Trigger = m_opJoystick.button(4);
    public static final int elevatorDownButton = 7;
    public static final int elevatorUpButton = 8;
    public static final Trigger resetYawTrigger = m_opJoystick.button(6);
    // public static final int goToTargetButton = 0; // TODO: change this
    public static final int overrideButton = 10; // TODO: fix this
    public static final int goToL1Button = 3;
    public static final int goToL2Button = 1;
    public static final int goToL3Button = 2;
    public static final int goToL4Button = 4;
    public static final int goToElevatorDownButton = 6;

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
    public static final double XControllerTolerance = 0.00; // 0.01
    public static final double XControllerIZone = 0.5;

    public static final double YControllerkP = 6.5;
    public static final double YControllerkI = 0.03;
    public static final double YControllerkD = 1.3;
    public static final double YControllerTolerance = 0.00; // 0.01
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

    // TODO: put actual values TODO: consider a field constants? doesn't go in drive
    public static final Translation2d blueReefCenter = new Translation2d(4.490323, -0.0001 + 4.02);
    public static final Translation2d redReefCenter = new Translation2d(13.059902, -0.0001 + 4.02);
    public static final Translation2d leftC2 = new Translation2d(1.2716, -0.16);
    public static final Translation2d rightC2 = new Translation2d(1.2716, 0.16);
    public static final Translation2d redUpperFeederCenter = new Translation2d(); // TODO: find these numbers
    public static final Translation2d redLowerFeederCenter = new Translation2d();
    public static final Translation2d blueUpperFeederCenter = new Translation2d();
    public static final Translation2d blueLowerFeederCenter = new Translation2d();
    public static final int autoAlignSectorCount = 6;
    public static final double autoAlignSectorRadius = 3; // TODO: change
    public static final double autoAlignSectorOffset = 30;
    public static final double autoAlignTolerance = Math.PI / 4;
    public static final double autoAlignDistanceTolerance = 10; // TODO: this is almost certainly the wrong number,

    // TODO: make comments with the corresponding april tags and red alliance
    public static final Pose2d[] leftAlignPoses = {
        new Pose2d(leftC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(0))
            .plus(blueReefCenter), new Rotation2d(Math.PI)), // C2
        new Pose2d(leftC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(Math.PI / 3))
            .plus(blueReefCenter), new Rotation2d(Math.PI * 4 / 3)), // C1
        new Pose2d(leftC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(Math.PI * 2 / 3))
            .plus(blueReefCenter), new Rotation2d(Math.PI * 5 / 3)), // F1
        new Pose2d(leftC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(Math.PI))
            .plus(blueReefCenter), new Rotation2d(0)), // F2
        new Pose2d(leftC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(Math.PI * 4 / 3))
            .plus(blueReefCenter), new Rotation2d(Math.PI / 3)), // F3
        new Pose2d(leftC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(Math.PI * 5 / 3))
            .plus(blueReefCenter), new Rotation2d(Math.PI * 2 / 3)), // C3

        new Pose2d(leftC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(0))
            .plus(redReefCenter), new Rotation2d(Math.PI)), // 7
        new Pose2d(leftC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(Math.PI / 3))
            .plus(redReefCenter), new Rotation2d(Math.PI * 4 / 3)),
        new Pose2d(leftC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(Math.PI * 2 / 3))
            .plus(redReefCenter), new Rotation2d(Math.PI * 5 / 3)),
        new Pose2d(leftC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(Math.PI))
            .plus(redReefCenter), new Rotation2d(0)),
        new Pose2d(leftC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(Math.PI * 4 / 3))
            .plus(redReefCenter), new Rotation2d(Math.PI / 3)),
        new Pose2d(leftC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(Math.PI * 5 / 3))
            .plus(redReefCenter), new Rotation2d(Math.PI * 2 / 3)),
    };
    public static final Pose2d[] rightAlignPoses = {
        new Pose2d(rightC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(0))
            .plus(blueReefCenter), new Rotation2d(Math.PI)), // C2
        new Pose2d(rightC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(Math.PI / 3))
            .plus(blueReefCenter), new Rotation2d(Math.PI * 4 / 3)), // C1
        new Pose2d(rightC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(Math.PI * 2 / 3))
            .plus(blueReefCenter), new Rotation2d(Math.PI * 5 / 3)), // F1
        new Pose2d(rightC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(Math.PI))
            .plus(blueReefCenter), new Rotation2d(0)), // F2
        new Pose2d(rightC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(Math.PI * 4 / 3))
            .plus(blueReefCenter), new Rotation2d(Math.PI / 3)), // F3
        new Pose2d(rightC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(Math.PI * 5 / 3))
            .plus(blueReefCenter), new Rotation2d(Math.PI * 2 / 3)), // C3

        new Pose2d(rightC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(0))
            .plus(redReefCenter), new Rotation2d(Math.PI)), // 7
        new Pose2d(rightC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(Math.PI / 3))
            .plus(redReefCenter), new Rotation2d(Math.PI * 4 / 3)),
        new Pose2d(rightC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(Math.PI * 2 / 3))
            .plus(redReefCenter), new Rotation2d(Math.PI * 5 / 3)),
        new Pose2d(rightC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(Math.PI))
            .plus(redReefCenter), new Rotation2d(0)),
        new Pose2d(rightC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(Math.PI * 4 / 3))
            .plus(redReefCenter), new Rotation2d(Math.PI / 3)),
        new Pose2d(rightC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(Math.PI * 5 / 3))
            .plus(redReefCenter), new Rotation2d(Math.PI * 2 / 3)),
    };
    public static final Pose2d[] leftFeederAlignPoses = {
        FlippingUtil.flipFieldPose(new Pose2d(1.642, 7.376, Rotation2d.fromDegrees(54.293))), // feeder sector 3
        FlippingUtil.flipFieldPose(new Pose2d(0.731, 1.310, Rotation2d.fromDegrees(-53.746))), // feeder sector 2
        new Pose2d(1.642, 7.376, Rotation2d.fromDegrees(-53.746)), // feeder sector 0
        new Pose2d(0.731, 1.310, Rotation2d.fromDegrees(54.293)), // feeder sector 1
    };
    public static final Translation2d[] feederAlignAngles = {
        // make sure these are the correct sectors (0,1,2,3) in that order
        new Translation2d(-0.64278761, 0.76604444), // feeder sector 0
        new Translation2d(-0.76604444, -0.64278761), // feeder sector 1
        new Translation2d(0.64278761, -0.76604444), // feeder sector 2
        new Translation2d(0.64278761, 0.76604444) // feeder sector 3
    };

    public static final Pose2d[] rightFeederAlignPoses = {
        FlippingUtil.flipFieldPose(new Pose2d(1.654, 0.626, Rotation2d.fromDegrees(54.293))), // feeder sector 3
        FlippingUtil.flipFieldPose(new Pose2d(0.707, 6.704, Rotation2d.fromDegrees(-53.746))), // feeder sector 2
        new Pose2d(0.707, 6.704, Rotation2d.fromDegrees(-53.746)), // feeder sector 0
        new Pose2d(1.654, 0.626, Rotation2d.fromDegrees(54.293)), // feeder sector 1
    };
  }

  public static class ClimberConstants {
    public static final double foldSpeed = 0.5;
    public static final double foldRunTime = 2; // TODO: Make this a number that makes sense

    // TODO: find actual values
    public static final double pinnerQuarterRotation = 10.5; // 42/4
    public static final double pinnerkP = 0.05;
    public static final double pinnerkI = 0.00001;
    public static final double pinnerkD = 0;
    public static final double pinnerkMinOutput = -1;
    public static final double pinnerkMaxOutPut = 1;

    public static final double pinnerVertical = 0.71;
    public static final double pinnerHorizontal = 0.95;

    public static final double climbModulekP = 0.2;
    public static final double climbModulekI = 0.001;
    public static final double climbModulekD = 0;
    public static final double climbModulekF = 0;

    public static final double climberUpAbsolute = 0.27;
    public static final double climberUpRelative = 0.0;
    public static final double climberDownRelative = 100.0;
    public static final double climberGearRatio = 243;
  }

  public static class IntakeConstants {
    public static final double intakeSpeed = 0.5;
  }

  public static class PoseEstimatorConstants {
    // TODO: this is all pasted from last year; PLEASE DON'T USE, FIND REAL VALUES
    public static final Matrix<N3, N1> statesStandardDev = VecBuilder.fill(0.001, 0.001, 0.005);
    public static final double visionXStandardDev = 0.0005; // TODO: adjust with framerate
    public static final double visionYStandardDev = 0.0005;
    public static final double visionHeadingStandardDev = 0.5;

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
        // TODO: update rotation for coral stations
        // 4, 5, 14, and 15 aren't updated and shouldn't be because they're weird
        new Pose2d(),
        new Pose2d(7.923198 + 8.775, -3.37068 + 4.02, new Rotation2d(Math.PI * 2 / 3)), // 1
        new Pose2d(7.923198 + 8.775, 3.37068 + 4.02, new Rotation2d(Math.PI * 2 / 3)), // 2
        new Pose2d(2.78681 + 8.775, 4.02961 + 4.02, new Rotation2d(Math.PI * 3 / 2)), // 3
        new Pose2d(0.50208 + 8.775, 2.111656 + 4.02, new Rotation2d(Math.PI)), // 4
        new Pose2d(0.50208 + 8.775, -2.111094 + 4.02, new Rotation2d(Math.PI * 3 / 2)), // 5
        new Pose2d(4.700446 + 8.775, -0.719682 + 4.02, new Rotation2d(Math.PI * 5 / 2)), // 6
        new Pose2d(5.116498 + 8.775, -0.0001 + 4.02, new Rotation2d(0)), // 7
        new Pose2d(4.700446 + 8.775, 0.719482 + 4.02, new Rotation2d(Math.PI / 3)), // 8
        new Pose2d(3.869358 + 8.775, 0.719482 + 4.02, new Rotation2d(Math.PI * 2 / 3)), // 9
        new Pose2d(3.453306 + 8.775, -0.0001 + 4.02, new Rotation2d(Math.PI)), // 10
        new Pose2d(3.869358 + 8.775, -0.719682 + 4.02, new Rotation2d(Math.PI * 4 / 3)), // 11
        new Pose2d(-7.922846 + 8.775, -3.37068 + 4.02, new Rotation2d(Math.PI / 3)), // 12
        new Pose2d(-7.922846 + 8.775, 3.37048 + 4.02, new Rotation2d(Math.PI)), // 13
        new Pose2d(-0.501728 + 8.775, 2.111656 + 4.02, new Rotation2d(0)), // 14
        new Pose2d(-0.501728 + 8.775, -2.111094 + 4.02, new Rotation2d(Math.PI * 2 / 3)), // 15
        new Pose2d(-2.786458 + 8.775, -4.02981 + 4.02, new Rotation2d(Math.PI / 2)), // 16
        new Pose2d(-4.700094 + 8.775, 0.719682 + 4.02, new Rotation2d(Math.PI * 4 / 3)), // 17
        new Pose2d(-5.1164 + 8.775, -0.00001 + 4.02, new Rotation2d(Math.PI)), // 18
        new Pose2d(-4.700094 + 8.775, 0.719482 + 4.02, new Rotation2d(Math.PI * 2 / 3)), // 19
        new Pose2d(-3.86926 + 8.775, 0.719482 + 4.02, new Rotation2d(Math.PI / 3)), // 20
        new Pose2d(-3.452954 + 8.775, -0.0001 + 4.02, new Rotation2d(0)), // 21
        new Pose2d(-3.86926 + 8.775, -0.719682 + 4.02, new Rotation2d(Math.PI * 5 / 3)) // 22
    };
  }

  public static class OuttakeConstants {
    public static final double outtakeSpeed = -0.6;
    public static final double loadSpeed = -0.3;
    public static final double scoreDelay = 0.5;
  }

}
