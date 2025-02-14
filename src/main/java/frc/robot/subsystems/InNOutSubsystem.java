package frc.robot.subsystems;

import frc.robot.BeamBreak;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OuttakeConstants;

import java.time.Instant;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class InNOutSubsystem extends SubsystemBase {
    TalonFX m_intakeMotor = new TalonFX(IDConstants.IntakeMotorID);
    TalonFX m_outtakeMotor = new TalonFX(IDConstants.outtakeMotorID);
    public TalonFX m_intakeFoldMotor = new TalonFX(IDConstants.IntakeFoldMotorID);
    Trigger m_fakeBeamBreak = ControllerConstants.m_driveJoystick.button(4);
    public String m_state = "empty";
    NetworkTable m_table;
    BeamBreak m_intakeBeamBreak;
    BeamBreak m_outtakeBeamBreak; // Checks to see if can score
    BeamBreak m_elevatorBeamBreak; // Make sure coral doesn't block elevator

    public InNOutSubsystem() {
        m_table = NetworkTableInstance.getDefault().getTable("InNOutSubsystem");
        m_outtakeBeamBreak = new BeamBreak(1); // TODO: find the real channel IDs
        m_elevatorBeamBreak = new BeamBreak(2);
        m_intakeBeamBreak = new BeamBreak(0); // Is in robot TODO: Find channel id and remove fakeBeamBreak

    }

    public void runIntake() {
        m_intakeMotor.set(IntakeConstants.intakeSpeed);
    }

    public void runOuttake() {
        m_outtakeMotor.set(OuttakeConstants.outtakeSpeed);
    }

    public void stopIntake() {
        m_intakeMotor.set(0);
        // LED (on intake motor?) red
        // LED (on shoot motor?) green
    }

    public void stopOuttake() {
        m_outtakeMotor.set(0);
    }

    public void periodic() {
        m_table.putValue("state", NetworkTableValue.makeString(m_state));
    }

    public boolean intakeHasCoral() {
        return m_elevatorBeamBreak.beamBroken() || m_intakeBeamBreak.beamBroken();
    }

    public boolean isLoaded() {
        return m_outtakeBeamBreak.beamBroken() && !m_elevatorBeamBreak.beamBroken();
    }

    public boolean isEmpty() {
        return !m_elevatorBeamBreak.beamBroken() && !m_intakeBeamBreak.beamBroken() && !m_outtakeBeamBreak.beamBroken();
    }

    public Command IntakeCoral() {
        return new ParallelCommandGroup(
                new StartEndCommand(this::runIntake, this::stopIntake),
                new StartEndCommand(this::runOuttake, this::stopOuttake));
    }

    public Command OuttakeCoral() {
        return new StartEndCommand(this::runOuttake, this::stopOuttake);
    }
}
