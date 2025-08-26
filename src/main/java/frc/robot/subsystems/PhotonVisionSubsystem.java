// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;

public class PhotonVisionSubsystem extends SubsystemBase {
  PhotonCamera camera;
  NetworkTable m_table;

  double startTime = Timer.getFPGATimestamp();
  double targetTimeStamp = Timer.getFPGATimestamp();

  public double yaw = 0;

  /** Creates a new PhotonVisionSubsystem. */
  public PhotonVisionSubsystem() {
    camera = new PhotonCamera("basketball_camera");
    m_table = NetworkTableInstance.getDefault().getTable("PhotonVisionSubsystem");
  }

  @Override
  public void periodic() {
    PhotonPipelineResult result = camera.getLatestResult();
    boolean hasTargets = result.hasTargets();
    m_table.putValue("Has Targets", NetworkTableValue.makeBoolean(hasTargets));
    if (hasTargets) {
      targetTimeStamp = Timer.getFPGATimestamp();
      PhotonTrackedTarget target = result.getBestTarget();
      yaw = target.getYaw();
      m_table.putValue("Yaw", NetworkTableValue.makeDouble(yaw));
      double pitch = target.getPitch();
      m_table.putValue("Pitch", NetworkTableValue.makeDouble(pitch));
      double area = target.getArea();
      m_table.putValue("Area", NetworkTableValue.makeDouble(area));
      double skew = target.getSkew();
      m_table.putValue("Skew", NetworkTableValue.makeDouble(skew));
      Transform3d pose = target.getBestCameraToTarget(); // might be wrong
      List<TargetCorner> corners = target.getDetectedCorners();
      int id = target.getDetectedObjectClassID();
      m_table.putValue("ID", NetworkTableValue.makeDouble(id));
      float confidence = target.getDetectedObjectConfidence();
      m_table.putValue("Confidence", NetworkTableValue.makeDouble(confidence));
    } else {
      if (Timer.getFPGATimestamp() - targetTimeStamp > LimelightConstants.targetForgetTime) {
        yaw = 0;
      }
    }
    if (Timer.getFPGATimestamp() - startTime > 1 && DriverStation.isEnabled()) {
      camera.takeInputSnapshot();
      camera.takeOutputSnapshot();
      startTime = Timer.getFPGATimestamp();
    }
  }
}
