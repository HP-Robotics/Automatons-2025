// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import com.pathplanner.lib.config.PIDConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class IDConstants {
    public static final int FLTurningMotorID = 0;
    public static final int FRTurningMotorID = 0;
    public static final int BRTurningMotorID = 0;
    public static final int BLTurningMotorID = 0;
    public static final int FLDriveMotorID = 0;
    public static final int FRDriveMotorID = 0;
    public static final int BRDriveMotorID = 0;
    public static final int BLDriveMotorID = 0;
    public static final int PigeonID = 0;
  }
  public static class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 5.0;
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
    public static final int FLAbsEncoder = 0;
    public static final int FRAbsEncoder = 0;
    public static final int BRAbsEncoder = 0;
    public static final int BLAbsEncoder = 0;
  }

  public static class DriveConstants {
    public static final double kMaxSpeed = 4.4; // meters per second
    public static final double kMaxAngularSpeed = Math.PI * 1.3; // 1/2 rotation per second //auto is 540
    public static final double kSlowSpeed = 2.0;
    public static final double kSlowAngularspeed = Math.PI / 2; // 1/4 rotation per second

    public static final double kWheelRadius = 0.0508 * (218.5 / 225.6); // This is a fudge factor
    public static final double kEncoderResolution = 1.0;

    public static final double driveGearRatio = 6.75;
    public static final double turningGearRatio = 540.0 / 35.0;

    public static final Translation2d kFrontLeftLocation = new Translation2d(0.308 - 0.038, 0.308);
    public static final Translation2d kFrontRightLocation = new Translation2d(0.308 - 0.038, -0.308);
    public static final Translation2d kBackLeftLocation = new Translation2d(-0.308, 0.308);
    public static final Translation2d kBackRightLocation = new Translation2d(-0.308, -0.308);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        kFrontLeftLocation, kFrontRightLocation, kBackRightLocation, kBackLeftLocation);

    public static final double drivekP = 5;
    public static final double drivekI = 10;
    public static final double drivekD = 0;
    public static final double drivekF = 0;

    public static final double turningkP = 1.8;
    public static final double turningkI = 1;
    public static final double turningkD = 0.008;

    public static final double turningControllerkP = 1.8;
    public static final double turningControllerkI = 0.3;
    public static final double turningControllerkD = 0.15;
    public static final double turningControllerTolerance = Math.toRadians(2);
    public static final double turningControllerIZone = 0.15;

    // Absolute encoder values that make the wheels point forward
    public static final double absEncoderForwardFL = 0.973;
    public static final double absEncoderForwardFR = 0.612;
    public static final double absEncoderForwardBR = 0.746;
    public static final double absEncoderForwardBL = 0.551;

    // public static final HolonomicPathFollowerConfig holonomicConfig = new
    // HolonomicPathFollowerConfig(
    // new PIDConstants(5.0, 0.0, 0.0),
    // new PIDConstants(5.0, 0.0, 0.0),
    // 4.5,
    // 0.4, // Distance from robot center to furthest module.
    // new ReplanningConfig());

    public static final double currentMax = 40.0;
    public static final double currentMin = -40.0;

    public static final double rampTimeTo300s = 0.01;

    public static final double currentLimit = 40.0;
    public static final double currentThreshold = 40.0;
    public static final double currentTimeThreshold = 0.04;

    public static final double driveToNoteSpeed = 0.5;

    public static final HolonomicPathFollowerConfig holonomicConfig = new HolonomicPathFollowerConfig(
      new PIDConstants(5.0, 0.0, 0.0),
      new PIDConstants(5.0, 0.0, 0.0),
      4.5,
      0.4, // Distance from robot center to furthest module.
      new ReplanningConfig());

  }

  public static final class PoseEstimatorConstants {

    public static final Matrix<N3, N1> statesStandardDev = VecBuilder.fill(0.001, 0.001, 0.005);
    public static final double visionXStandardDev = 0.005; // TODO: adjust with framerate
    public static final double visionYStandardDev = 0.005;
    public static final double visionHeadingStandardDev = 0.05;

    public static final double maxAcceptableSkew = Math.PI / 3;
    public static final double maxAcceptableDistance = 5.0;
  }
}
