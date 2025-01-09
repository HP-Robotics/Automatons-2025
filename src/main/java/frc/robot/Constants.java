// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class LimelightConstants {
    public static final double inToM = 0.0254;
    public static final Pose2d aprilTagList[] = { // 0 is empty, april tag number is that number in list
        // TODO: These are values from last year so get the ones from this year
        new Pose2d(),
        new Pose2d(593.68 * inToM, 9.68 * inToM, new Rotation2d(Math.PI * 2 / 3)), // 1
        new Pose2d(637.21 * inToM, 34.79 * inToM, new Rotation2d(Math.PI * 2 / 3)), // 2
        new Pose2d(652.73 * inToM, 196.17 * inToM, new Rotation2d(Math.PI)), // 3
        new Pose2d(652.73 * inToM, 218.42 * inToM, new Rotation2d(Math.PI)), // 4
        new Pose2d(578.77 * inToM, 323.00 * inToM, new Rotation2d(Math.PI * 3 / 2)), // 5
        new Pose2d(72.5 * inToM, 323.00 * inToM, new Rotation2d(Math.PI * 3 / 2)), // 6
        new Pose2d(-1.5 * inToM, 218.42 * inToM, new Rotation2d(0)), // 7
        new Pose2d(-1.5 * inToM, 196.17 * inToM, new Rotation2d(0)), // 8
        new Pose2d(14.02 * inToM, 34.79 * inToM, new Rotation2d(Math.PI / 3)), // 9
        new Pose2d(57.54 * inToM, 9.68 * inToM, new Rotation2d(Math.PI / 3)), // 10
        new Pose2d(468.69 * inToM, 146.19 * inToM, new Rotation2d(Math.PI * 5 / 3)), // 11
        new Pose2d(468.69 * inToM, 177.10 * inToM, new Rotation2d(Math.PI / 3)), // 12
        new Pose2d(441.74 * inToM, 161.62 * inToM, new Rotation2d(Math.PI)), // 13
        new Pose2d(209.48 * inToM, 161.62 * inToM, new Rotation2d(0)), // 14
        new Pose2d(182.73 * inToM, 177.10 * inToM, new Rotation2d(Math.PI * 2 / 3)), // 15
        new Pose2d(182.73 * inToM, 146.19 * inToM, new Rotation2d(Math.PI * 4 / 3)) // 16
    };
  }

  public static class PoseEstimatorConstants {
    //TODO: this is all pasted from last year; PLEASE DON'T USE, FIND REAL VALUES
    public static final Matrix<N3, N1> statesStandardDev = VecBuilder.fill(0.001, 0.001, 0.005);
    public static final double visionXStandardDev = 0.005; //TODO: adjust with framerate
    public static final double visionYStandardDev = 0.005;
    public static final double visionHeadingStandardDev = 0.05;

    public static final double maxAcceptableSkew = Math.PI / 3;
    public static final double maxAcceptableDistance = 5.0; 
  }
}
