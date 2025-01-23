// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.LimelightConstants;

public class LimelightSubsystem extends SubsystemBase {
  NetworkTableEntry botposeBlue; // Red will be mirrored from this
  CommandJoystick m_driveJoystick;

  public final Field2d m_field = new Field2d();
  NetworkTable m_table;
  public double tx;
  public double ty;
  public double tz;
  public double rx;
  public double ry;
  public double rz;
  public boolean m_seeingAprilTag;
  public boolean m_aprilTagSeen;
  public int m_targetAprilTagID;
  final DoubleArraySubscriber limeSub;
  private double[] defaultValues = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  public Pose2d m_visionPose2d = new Pose2d();
  public Pose2d m_targetPoseRelative;

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable poseEstimatorTable = inst.getTable("pose-estimator-table");

  StructPublisher<Pose2d> publisher;

  PoseEstimatorSubsystem m_poseEstimator;

  public LimelightSubsystem(PoseEstimatorSubsystem poseEstimatorSubsystem) {
    Shuffleboard.getTab("shuffleboard")
        .add("Pose2d", m_field)
        .withWidget(BuiltInWidgets.kField);
    m_table = NetworkTableInstance.getDefault().getTable("limelight-bite");
    botposeBlue = m_table.getEntry("botpose_wpiblue");
    m_poseEstimator = poseEstimatorSubsystem;
    publisher = poseEstimatorTable.getStructTopic("AprilTag Pose", Pose2d.struct).publish();
    limeSub = m_table.getDoubleArrayTopic("botpose_wpiblue").subscribe(defaultValues);

    m_aprilTagSeen = false;
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

  @Override
  public void periodic() {
    double[] botPose = null;
    double latency = 0;
    double timeStamp = 0;
    m_seeingAprilTag = m_table.getEntry("tv").getDouble(0) != 0;
    botPose = botposeBlue.getDoubleArray(defaultValues);
    tx = m_table.getEntry("tx").getDouble(0);
    ty = m_table.getEntry("ty").getDouble(0);

    m_targetPoseRelative = new Pose2d(tx, ty, new Rotation2d());

    m_field.setRobotPose(m_targetPoseRelative);

    if (m_seeingAprilTag) {
      m_targetAprilTagID = (int) m_table.getEntry("tid").getInteger(0);
      for (TimestampedDoubleArray tsValue : limeSub.readQueue()) {
        botPose = tsValue.value;
        timeStamp = Timer.getFPGATimestamp();
        double botPosX = botPose[0];
        double botPosY = botPose[1];
        // double botPosZ = botpose[2];
        // double botRotX = botpose[3];
        // double botRotY = botpose[4]; // TODO: Store botPosZ, botRotX, and botRotY somewhere (Store 3D)
        double botRotZ = botPose[5];
        latency = botPose[6];
        timeStamp -= latency / 1000;
        m_targetPoseRelative = new Pose2d(botPosX, botPosY, new Rotation2d(Math.toRadians(botRotZ)));
        m_visionPose2d = m_targetPoseRelative;
        if (m_poseEstimator != null && 0 <= m_targetAprilTagID && m_targetAprilTagID <= 22) {
          if (botPosX != 0 || botPosY != 0 || botRotZ != 0) {
            publisher.set(m_targetPoseRelative);
            Rotation2d relativeSkew = Rotation2d.fromDegrees(getAngleToPose(m_targetPoseRelative, LimelightConstants.aprilTagList[m_targetAprilTagID])-180);
            double absoluteSkew = Math.abs(LimelightConstants.aprilTagList[m_targetAprilTagID].getRotation()
                .minus(relativeSkew)
                .getRadians()
            );
            absoluteSkew = Math.abs(absoluteSkew);
            m_poseEstimator.updateVision(m_targetPoseRelative, timeStamp,
                getDistanceToPose(m_targetPoseRelative, LimelightConstants.aprilTagList[m_targetAprilTagID]), absoluteSkew);
            // System.out.println("saw apriltag: " + timeStamp + " latency: " + latency);
          }
        }
      }
    }
  }
}