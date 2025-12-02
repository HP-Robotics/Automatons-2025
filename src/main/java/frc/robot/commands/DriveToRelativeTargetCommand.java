// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Function;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot.APResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class DriveToRelativeTargetCommand extends Command {
  DriveSubsystem m_driveSubsystem;
  Transform2d m_targetTrans;
  PoseEstimatorSubsystem m_poseEstimatorSubsystem;
  int m_allianceVelocityMultiplier;
  APTarget m_target;

  Function<Rotation2d, Double> m_pidRotation = (Rotation2d targetAngle) -> {
    double output = m_driveSubsystem.m_rotationController.calculate(m_poseEstimatorSubsystem.getPose().getRotation().getRadians(),
        targetAngle.getRadians());
    return m_driveSubsystem.m_rotationController.atSetpoint() ? 0 : output;
  };

  public DriveToRelativeTargetCommand(DriveSubsystem driveSubsystem, PoseEstimatorSubsystem poseEstimatorSubsystem,
      Transform2d targetTrans) {
    m_driveSubsystem = driveSubsystem;
    m_poseEstimatorSubsystem = poseEstimatorSubsystem;
    m_targetTrans = targetTrans;
  }

  @Override
  public void initialize() {
    System.out.println("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
    Pose2d targetPose = m_driveSubsystem.getPose().plus(m_targetTrans);
    m_target = new APTarget(targetPose);
  }

  @Override
  public void execute() {
    APResult result = LimelightConstants.kAutopilot.calculate(m_driveSubsystem.getPose(),
        m_driveSubsystem.getCurrentSpeeds(), m_target);
    double rot = m_pidRotation.apply(result.targetAngle());
    double vx = result.vx().in(MetersPerSecond);
    double vy = result.vy().in(MetersPerSecond);
    System.out.println(vx);
    System.out.println(vy);
    m_driveSubsystem.drive(
        vx,
        vy,
        rot * DriveConstants.kMaxAngularSpeed,
        true);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
