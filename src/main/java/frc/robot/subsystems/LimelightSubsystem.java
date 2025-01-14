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
import edu.wpi.first.networktables.NetworkTableValue;
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
  NetworkTableEntry botposeBlue;
  CommandJoystick m_driveJoystick;

  public final Field2d m_field = new Field2d();
  NetworkTable m_table;
  public double tx;
  public double ty;
  public double tz;
  public double rx;
  public double ry;
  public double rz;
  public double tv;
  public boolean m_sawAprilTag;
  public boolean m_aprilTagSeen;
  public int m_targetAprilTagID;
  final DoubleArraySubscriber limeSub;
  private double[] defaultValues = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  public Pose2d m_visionPose2d = new Pose2d();
  public Pose2d m_robotPose;

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

  public double getDistanceTo(Pose2d robot, Pose2d fieldpose) {
    double turnToSpeakerA = fieldpose.getX() - robot.getX();
    double turnToSpeakerB = fieldpose.getY() - robot.getY();
    double distanceToSpeaker = Math.sqrt(Math.pow(turnToSpeakerA, 2) + Math.pow(turnToSpeakerB, 2));

    return distanceToSpeaker;
  }

  public double getAngleTo(Pose2d robot, Pose2d fieldpose) {
    double turnToSpeakerA = fieldpose.getX() - robot.getX();
    double turnToSpeakerB = fieldpose.getY() - robot.getY();
    double angleToSpeaker = Math.atan2(turnToSpeakerB, turnToSpeakerA);

    return Math.toDegrees(angleToSpeaker);
  }

  @Override
  public void periodic() {
    double[] entries = new double[6];
        entries[0] = 0; //m_poseEstimator.getPose().getRotation().getDegrees();
        entries[1] = 0;
        entries[2] = 0;
        entries[3] = 0;
        entries[4] = 0;
        entries[5] = 0;
    double[] botpose = null;
    double latency = 0;
    double timeStamp = 0;
    double botposeEmpty[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    m_sawAprilTag = m_table.getEntry("tv").getDouble(0) != 0;
    botpose = botposeBlue.getDoubleArray(botposeEmpty);
    tx = m_table.getEntry("tx").getDouble(0);
    ty = m_table.getEntry("ty").getDouble(0);
    tv = m_table.getEntry("tv").getDouble(0);

    m_robotPose = new Pose2d(tx, ty, new Rotation2d());

    m_field.setRobotPose(m_robotPose);

    if (m_sawAprilTag) {
      m_targetAprilTagID = (int) m_table.getEntry("tid").getInteger(0);
      for (TimestampedDoubleArray tsValue : limeSub.readQueue()) {
        botpose = tsValue.value;
        timeStamp = Timer.getFPGATimestamp();
        double tx = botpose[0];
        double ty = botpose[1];
        // double tz = botpose[2];
        // double rx = botpose[3];
        // double ry = botpose[4]; // TODO: Store tz, rx, and ry somewhere (Store 3D)
        double rz = botpose[5];
        latency = botpose[6];
        timeStamp -= latency / 1000;
        m_robotPose = new Pose2d(tx, ty, new Rotation2d(Math.toRadians(rz)));
        m_visionPose2d = m_robotPose;
        if (m_poseEstimator != null && 0 <= m_targetAprilTagID && m_targetAprilTagID <= 22) {
          if (botpose[0] != 0 || botpose[1] != 0 || botpose[5] != 0) {
            publisher.set(m_robotPose);
            double skew = LimelightConstants.aprilTagList[m_targetAprilTagID].getRotation()
                .minus(Rotation2d
                    .fromDegrees(getAngleTo(m_robotPose, LimelightConstants.aprilTagList[m_targetAprilTagID])-180))
                .getRadians();
            skew = Math.abs(skew);
            m_poseEstimator.updateVision(m_robotPose, timeStamp,
                getDistanceTo(m_robotPose, LimelightConstants.aprilTagList[m_targetAprilTagID]), skew);
            // System.out.println("saw apriltag: " + timeStamp + " latency: " + latency);
          }
        }
      }
    }
  }
}