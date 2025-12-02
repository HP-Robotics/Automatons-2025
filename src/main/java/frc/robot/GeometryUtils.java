package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class GeometryUtils {
    public static boolean isNearTargetAngle(double angle, double targetAngle, double tolerance) {
    return Math.abs(angle - targetAngle) <= tolerance;
  }

  public static boolean isNearTargetAngle(Translation2d a, Translation2d b, double tolerance) {
    return Math.abs(Math.acos((a.getX() * b.getX() + a.getY() * b.getY())
        / (a.getNorm() * b.getNorm()))) <= tolerance;
  }

  public static double getAngleBetweenVectors(Translation2d a, Translation2d b) {
    return Math.acos((a.getX() * b.getX() + a.getY() * b.getY())
        / (a.getNorm() * b.getNorm()));
  }

  public static double getDistanceToPose(Pose2d robot, Pose2d fieldPose) {
    double distX = fieldPose.getX() - robot.getX();
    double distY = fieldPose.getY() - robot.getY();

    return Math.sqrt(Math.pow(distX, 2) + Math.pow(distY, 2));
  }

  public static double getAngleBetweenPoses(Pose2d pose1, Pose2d pose2) {
    double distX = pose2.getX() - pose1.getX();
    double distY = pose2.getY() - pose1.getY();
    double angleRadians = Math.atan2(distY, distX);

    return Math.toDegrees(angleRadians);
  }
}
