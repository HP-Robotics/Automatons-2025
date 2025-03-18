// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.LimelightConstants;

public class LimelightSubsystem extends SubsystemBase {
  CommandJoystick m_driveJoystick;

  NetworkTable m_leftTable;
  NetworkTable m_rightTable;
  public int targetAprilTagID;
  private final DoubleArraySubscriber m_leftSub;
  private final DoubleArraySubscriber m_rightSub;
  private double[] defaultValues = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable poseEstimatorTable = inst.getTable("pose-estimator-table");

  StructPublisher<Pose2d> publisher;

  PoseEstimatorSubsystem m_poseEstimator;

  public LimelightSubsystem(PoseEstimatorSubsystem poseEstimatorSubsystem) {
    m_leftTable = NetworkTableInstance.getDefault().getTable("limelight-shpwrte");
    m_rightTable = NetworkTableInstance.getDefault().getTable("limelight-kite");
    m_leftTable.getEntry("imumode_set").setDouble(0);
    m_rightTable.getEntry("imumode_set").setDouble(0); // 4 is scary
    m_poseEstimator = poseEstimatorSubsystem;
    publisher = poseEstimatorTable.getStructTopic("AprilTag Pose", Pose2d.struct).publish();
    m_leftSub = m_leftTable.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(defaultValues);
    m_rightSub = m_rightTable.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(defaultValues);
  }

  public double getDistanceToPose(Pose2d robot, Pose2d fieldPose) {
    double distX = fieldPose.getX() - robot.getX();
    double distY = fieldPose.getY() - robot.getY();

    return Math.sqrt(Math.pow(distX, 2) + Math.pow(distY, 2));
  }

  public double getAngleToPose(Pose2d robot, Pose2d fieldPose) {
    double distX = fieldPose.getX() - robot.getX();
    double distY = fieldPose.getY() - robot.getY();
    double angleRadians = Math.atan2(distY, distX);

    return Math.toDegrees(angleRadians);
  }

  public void setThrottle(int throttle) {
    m_rightTable.getEntry("throttle_set").setDouble(throttle);
  }

  public static Pose2d[] getFieldTags(AprilTagFieldLayout field) {
    Pose2d[] output = new Pose2d[field.getTags().size()];
    for (int i = 0; i < field.getTags().size(); i++) {
      output[i] = field.getTagPose(i).isPresent() ? field.getTagPose(i).get().toPose2d() : new Pose2d();
    }
    return output;
  }

  public void doLimelight(NetworkTable limelightTable, DoubleArraySubscriber botPoseSub) {
    double[] robotOrientationEntries = new double[6];
    if (m_poseEstimator != null) {
    robotOrientationEntries[0] = m_poseEstimator.getPose().getRotation().getDegrees();
    } else {
      robotOrientationEntries[0] = 0;
    }
    robotOrientationEntries[1] = 0;
    robotOrientationEntries[2] = 0;
    robotOrientationEntries[3] = 0;
    robotOrientationEntries[4] = 0;
    robotOrientationEntries[5] = 0;

    limelightTable.getEntry("robot_orientation_set").setDoubleArray(robotOrientationEntries);

    double[] botPose = null;
    double latency = 0;
    double timeStamp = 0;

    for (TimestampedDoubleArray tsValue : botPoseSub.readQueue()) {
      botPose = tsValue.value;
      if ((int) botPose[7] == 0) {
        continue;
      }
      timeStamp = Timer.getFPGATimestamp(); // TODO: tsValue.timestamp; test this
      double botPosX = botPose[0];
      double botPosY = botPose[1];
      // double botPosZ = botpose[2];
      // double botRotX = botpose[3];
      // double botRotY = botpose[4]; // TODO: Store botPosZ, botRotX, and botRotY
      // somewhere (Store 3D)
      double botRotZ = botPose[5];
      latency = botPose[6];
      targetAprilTagID = (int) botPose[11];

      timeStamp -= latency / 1000;
      Pose2d targetPoseRelative = new Pose2d(botPosX, botPosY, new Rotation2d(Math.toRadians(botRotZ)));
      if (m_poseEstimator != null && 0 <= targetAprilTagID
          && targetAprilTagID < LimelightConstants.aprilTagList.length) {
        limelightTable.putValue("botPosX", NetworkTableValue.makeDouble(botPosX));
        limelightTable.putValue("botPosY", NetworkTableValue.makeDouble(botPosY));
        limelightTable.putValue("botPosZ", NetworkTableValue.makeDouble(botRotZ));
        if (botPosX != 0 || botPosY != 0 || botRotZ != 0) {
          publisher.set(targetPoseRelative);
          Rotation2d aprilTagToRobotAngle = Rotation2d.fromDegrees(
              getAngleToPose(targetPoseRelative, LimelightConstants.aprilTagList[targetAprilTagID]) - 180);
          limelightTable.putValue("Angle to pose", NetworkTableValue
              .makeDouble(getAngleToPose(targetPoseRelative, LimelightConstants.aprilTagList[targetAprilTagID]) - 180));
          double angleDiff = Math.abs(LimelightConstants.aprilTagList[targetAprilTagID].getRotation()
              .minus(aprilTagToRobotAngle)
              .getRadians());
          m_poseEstimator.updateVision(targetPoseRelative, timeStamp,
              getDistanceToPose(targetPoseRelative, LimelightConstants.aprilTagList[targetAprilTagID]),
              angleDiff);
          // System.out.println("saw apriltag: " + timeStamp + " latency: " + latency);
        }
      }
    }
  }

  @Override
  public void periodic() {
    doLimelight(m_rightTable, m_rightSub);
  }
}