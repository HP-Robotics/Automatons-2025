// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BeamBreak;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.OuttakeConstants;

public class OuttakeSubsystem extends SubsystemBase {
  TalonFX m_outtakeMotor = new TalonFX(IDConstants.outtakeMotorID);
  BeamBreak elevatorBeamBreak = new BeamBreak(0);
  BeamBreak outtakeBeamBreak = new BeamBreak(0); // TODO: Find channel IDs
  public String m_state = "empty";
  NetworkTable m_table;

  /** Creates a new OuttakeSubsystem. */
  public OuttakeSubsystem() {
    m_table = NetworkTableInstance.getDefault().getTable("OuttakeSubsystem");
  }

  public void loadOuttake() {
    m_outtakeMotor.set(OuttakeConstants.loadSpeed);
    if (IntakeSubsystem.isLoaded()) {
      m_outtakeMotor.set(0);
      m_state = "loaded";
    }
  }

  public void runOuttake() {
    if (m_state == "loaded") {
      m_outtakeMotor.set(OuttakeConstants.outtakeSpeed);
      m_state = "shoot";
    } else if (m_state != "loaded") {
      m_outtakeMotor.set(0);
      m_state = "empty";
    }
  }

  public void stopOuttake() {
    m_outtakeMotor.set(0);
  }

  @Override
  public void periodic() {
    m_table.putValue("state", NetworkTableValue.makeString(m_state));
  }

  public Command LoadOuttake() {
    return new InstantCommand(this::loadOuttake);
  }

  public Command RunOuttake() {
    return new InstantCommand(this::runOuttake);
  }
}