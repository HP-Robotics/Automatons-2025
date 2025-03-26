// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.LimelightSubsystem;

public final class Constants {

  public static class SubsystemConstants {
    public static final boolean useDrive = true;
    public static final boolean useIntake = true;
    public static final boolean useOuttake = true;
    public static final boolean useDataManager = true;
    public static final boolean useLimelight = true;
    public static final boolean useClimber = true;
    public static final boolean useLED = true;
    public static final boolean usePoseEstimator = true;
    public static final boolean useElevator = true;
    public static final boolean useDealginator = true;
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

    public static final int ElevatorMotor1ID = 60;
    public static final int ElevatorMotor2ID = 61;

    public static final int climbMotorID = 11;
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
    public static final double inchesToRotations = 5 / (Math.PI * 1.504);
    public static final double L4Position = (71.82 + 3.5) * inchesToRotations; // 42.5 if low ceiling
    public static final double L3Position = (48.6670401153 + 3) * inchesToRotations; // 37.7 if low ceiling
    public static final double L2Position = (34.02 + 1) * inchesToRotations;
    public static final double L1Position = (18.9 + 5) * inchesToRotations;
    public static final double elevatorDownPosition = (2.95) * inchesToRotations; // testing
    public static final double elevatorTravelPosition = 4.457 * inchesToRotations;
    public static final double dealginatePosition = elevatorTravelPosition + 2 * inchesToRotations;
    public static final double bottomPosition = (0.945 + 1) * inchesToRotations;
    // TODO: this might be right but should be checked with the other two
    public static final double kP = 1.5;// TODO: tune these more
    public static final double kI = 0.3;
    public static final double kD = 0;

    public static final double errorTolerance = 0.5;

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

    public static final double elevatorWiggleAmount = 0.5 * inchesToRotations;
    public static final double elevatorWiggleWait = 0.1;
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
    public static final Trigger outtakeTrigger = m_driveJoystick.button(4);
    public static final int leftAlignButton = 5;
    public static final int rightAlignButton = 6;
    public static final int intakeFoldButton = 7;
    public static final int intakeFoldDualKeyButton = 8;
    public static final Trigger closePinnerTrigger = m_driveJoystick.axisGreaterThan(3, 0.2);
    public static final Trigger resetYawTrigger = m_driveJoystick.povDownRight();

    // OPERATOR BUTTONS
    // public static final Trigger elevatorL3Trigger = m_opJoystick.button(3);
    // public static final Trigger elevatorL4Trigger = m_opJoystick.button(4);
    public static final Trigger elevatorDownButton = m_opJoystick.button(7);
    public static final Trigger elevatorUpButton = m_opJoystick.button(8);
    public static final Trigger intakeTrigger = m_opJoystick.button(5);
    public static final Trigger dealginateButton = m_opJoystick.povUp();

    public static final Trigger overrideButton = m_opJoystick.povDown();
    public static final Trigger goToL1Button = m_opJoystick.button(3);
    public static final Trigger goToL2Button = m_opJoystick.button(1);
    public static final Trigger goToL3Button = m_opJoystick.button(2);
    public static final Trigger goToL4Button = m_opJoystick.button(4);
    public static final Trigger goToElevatorDownButton = m_opJoystick.button(6);

    public static final Trigger opIntakeFold = m_opJoystick.povLeft();
    public static final Trigger opDualKeyIntakeFoldTrigger = m_opJoystick.axisGreaterThan(3, 0.2);
    public static final Trigger opTripleKeyIntakeFoldTrigger = m_opJoystick.axisGreaterThan(2, 0.2);

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
    // TODO: make these the same order as the swerve setpoint generator (we think
    // they're in the right order but aren't sure)

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
    public static final double rotationControllerkI = 0.1;
    public static final double rotationControllerkD = 0.15;
    public static final double rotationControllerTolerance = Math.toRadians(0.5);
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

    public static final double autoAlignJoystickTolerance = Math.PI / 4;
    public static final double poseDistanceTolerance = 0.02;
    public static final double poseAngleTolerance = Math.PI / 60;
  };

  public static class FieldConstants {
    public static final Translation2d blueReefCenter = new Translation2d(4.490323, 4.02);
    public static final Translation2d redReefCenter = new Translation2d(13.059902, 4.02);
    public static final Translation2d leftC2 = new Translation2d(1.2716 + 0.03, -0.18); // + 0.04 untested
    public static final Translation2d rightC2 = new Translation2d(1.2716 + 0.03, 0.16);
    public static final Translation2d redUpperFeederCenter = new Translation2d(); // TODO: find these numbers
    public static final Translation2d redLowerFeederCenter = new Translation2d();
    public static final Translation2d blueUpperFeederCenter = new Translation2d();
    public static final Translation2d blueLowerFeederCenter = new Translation2d();
    public static final int autoAlignSectorCount = 6;
    public static final double autoAlignSectorRadius = 3;
    public static final double autoAlignSectorOffset = 30;
    public static final double autoAlignFeederRange = 10; // TODO: this is almost certainly the wrong number,
    public static final double autoAlignDistanceMultiplier = 2;

    public static final Pose2d[] leftAlignPoses = {
        new Pose2d(leftC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(0))
            .plus(blueReefCenter), new Rotation2d(Math.PI)), // C2, 21
        new Pose2d(leftC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(Math.PI / 3))
            .plus(blueReefCenter), new Rotation2d(Math.PI * 4 / 3)), // C1, 20
        new Pose2d(leftC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(Math.PI * 2 / 3))
            .plus(blueReefCenter), new Rotation2d(Math.PI * 5 / 3)), // F1, 19
        new Pose2d(leftC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(Math.PI))
            .plus(blueReefCenter), new Rotation2d(0)), // F2, 18
        new Pose2d(leftC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(Math.PI * 4 / 3))
            .plus(blueReefCenter), new Rotation2d(Math.PI / 3)), // F3, 17
        new Pose2d(leftC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(Math.PI * 5 / 3))
            .plus(blueReefCenter), new Rotation2d(Math.PI * 2 / 3)), // C3, 22

        new Pose2d(leftC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(0))
            .plus(redReefCenter), new Rotation2d(Math.PI)), // F2, 7
        new Pose2d(leftC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(Math.PI / 3))
            .plus(redReefCenter), new Rotation2d(Math.PI * 4 / 3)), // F1, 8
        new Pose2d(leftC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(Math.PI * 2 / 3))
            .plus(redReefCenter), new Rotation2d(Math.PI * 5 / 3)), // C1, 9
        new Pose2d(leftC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(Math.PI))
            .plus(redReefCenter), new Rotation2d(0)), // C2, 10
        new Pose2d(leftC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(Math.PI * 4 / 3))
            .plus(redReefCenter), new Rotation2d(Math.PI / 3)), // C3, 11
        new Pose2d(leftC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(Math.PI * 5 / 3))
            .plus(redReefCenter), new Rotation2d(Math.PI * 2 / 3)), // F3, 6
    };
    public static final Pose2d[] rightAlignPoses = {
        new Pose2d(rightC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(0))
            .plus(blueReefCenter), new Rotation2d(Math.PI)), // C2, 21
        new Pose2d(rightC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(Math.PI / 3))
            .plus(blueReefCenter), new Rotation2d(Math.PI * 4 / 3)), // C1, 20
        new Pose2d(rightC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(Math.PI * 2 / 3))
            .plus(blueReefCenter), new Rotation2d(Math.PI * 5 / 3)), // F1, 19
        new Pose2d(rightC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(Math.PI))
            .plus(blueReefCenter), new Rotation2d(0)), // F2, 18
        new Pose2d(rightC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(Math.PI * 4 / 3))
            .plus(blueReefCenter), new Rotation2d(Math.PI / 3)), // F3, 17
        new Pose2d(rightC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(Math.PI * 5 / 3))
            .plus(blueReefCenter), new Rotation2d(Math.PI * 2 / 3)), // C3, 22

        new Pose2d(rightC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(0))
            .plus(redReefCenter), new Rotation2d(Math.PI)), // F2, 7
        new Pose2d(rightC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(Math.PI / 3))
            .plus(redReefCenter), new Rotation2d(Math.PI * 4 / 3)), // F1, 8
        new Pose2d(rightC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(Math.PI * 2 / 3))
            .plus(redReefCenter), new Rotation2d(Math.PI * 5 / 3)), // C1, 9
        new Pose2d(rightC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(Math.PI))
            .plus(redReefCenter), new Rotation2d(0)), // C2, 10
        new Pose2d(rightC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(Math.PI * 4 / 3))
            .plus(redReefCenter), new Rotation2d(Math.PI / 3)), // C3, 11
        new Pose2d(rightC2.plus(new Translation2d()).rotateAround(new Translation2d(), new Rotation2d(Math.PI * 5 / 3))
            .plus(redReefCenter), new Rotation2d(Math.PI * 2 / 3)), // F3, 6

    };
    public static final Pose2d[] leftFeederAlignPoses = {
        FlippingUtil.flipFieldPose(new Pose2d(0.731, 1.310, Rotation2d.fromDegrees(54.293))), // feeder sector 3
        FlippingUtil.flipFieldPose(new Pose2d(0.707, 6.704, Rotation2d.fromDegrees(-53.746))), // feeder sector 2
        new Pose2d(0.707, 6.704, Rotation2d.fromDegrees(-53.746)), // feeder sector 0
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
        FlippingUtil.flipFieldPose(new Pose2d(1.642, 7.376, Rotation2d.fromDegrees(-53.746))), // feeder sector 2
        new Pose2d(1.642, 7.376, Rotation2d.fromDegrees(-53.746)), // feeder sector 0
        new Pose2d(1.654, 0.626, Rotation2d.fromDegrees(54.293)), // feeder sector 1
    };
  }

  public static class ClimberConstants {
    public static final double foldSpeed = 0.1;
    public static final double foldRunTime = 2;

    public static final double climbModulekP = 0.4;
    public static final double climbModulekI = 0.001;
    public static final double climbModulekD = 0.04;
    public static final double climbModulekF = 0;

    public static final double climberGearRatio = 473.5; // 243;

    public static final double encoderModulusTolerance = 0.2;
    public static final double degreesToRelativeRotations = climberGearRatio / 360;
    public static final double climberStartAbsolute = 0.8;
    public static final double climberUpRelative = 0.0;
    public static final double climberDownRelative = 118.0 * degreesToRelativeRotations; // TEMP TEMP 137.78 *
                                                                                         // degreesToRelativeRotations;

    public static final double offsetErrorThreshold = 5;
  }

  public static class IntakeConstants {
    public static final double intakeSpeed = 0.5;
  }

  public static class PoseEstimatorConstants {
    public static final Matrix<N3, N1> statesStandardDev = VecBuilder.fill(0.001, 0.001, 0.005);
    public static final double visionXStandardDev = 0.0005; // TODO: adjust with framerate
    public static final double visionYStandardDev = 0.0005;
    public static final double visionHeadingStandardDev = 0.5;

    public static final double maxAcceptableSkew = Math.PI / 3; // TODO: decrease this or only check one sector
    public static final double maxAcceptableDistance = 5.0;
    public static final double skewWeight = 1.0 / maxAcceptableSkew;
    public static final double headingWeight = 180.0 / Math.PI;
    public static final double distanceWeight = 1.0 / 10.0;
  }

  public static final class PathplannerPIDConstants {
    public static final PIDConstants translationConstants = new PIDConstants(5, 0, 0);
    public static final PIDConstants rotationConstants = new PIDConstants(5, 0, 0);
  }

  public static class LimelightConstants {
    public static final double inToM = 0.0254;
    public static final AprilTagFieldLayout field = AprilTagFieldLayout
        .loadField(AprilTagFields.k2025ReefscapeWelded);
    public static final Pose2d aprilTagList[] = LimelightSubsystem.getFieldTags(field);
    public static final int disabledThrottle = 200;
    public static final double imuAssist = 0.005;
  }

  public static class OuttakeConstants {
    public static final double outtakeSpeed = -0.6;
    public static final double loadSpeed = -0.5;
    public static final double scoreDelay = 0.2;
  }

  public static class LEDConstants {
    public static final int port = 0;
    public static final int length = 60;
    public static final int middleFirstIndex = 10;
    public static final int middleLastIndex = 49;
    public static final Time blinkInterval = Seconds.of(0.5);

    public static final LEDPattern noCoralPattern = LEDPattern.solid(Color.kRed);
    public static final LEDPattern coralProcessingPattern = LEDPattern.solid(new Color(255, 32, 0))
        .blink(blinkInterval);
    public static final LEDPattern coralReadyPattern = LEDPattern.solid(Color.kWhite);
    public static final LEDPattern cagePattern = LEDPattern.solid(Color.kBlue);
    public static final LEDPattern elevatorMovingPattern = LEDPattern.solid(Color.kRed).blink(blinkInterval);
    public static final LEDPattern autoAlignReadyPattern = LEDPattern.solid(Color.kGreen);
    public static final LEDPattern autoAligningPattern = LEDPattern.solid(new Color(255, 32, 0)).blink(blinkInterval);
    public static final LEDPattern noAutoAlignPattern = LEDPattern.solid(Color.kBlack);
    public static final LEDPattern defaultSidePattern = LEDPattern.solid(Color.kRed).blink(blinkInterval);
    public static final LEDPattern defaultMiddlePattern = noCoralPattern;
    public static final Optional<LEDPattern> defaultOverridePattern = Optional.empty();

  }
}
