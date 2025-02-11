package frc.robot.subsystems;

import frc.robot.BeamBreak;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class IntakeSubsystem extends SubsystemBase {
    TalonFX m_intakeMotor = new TalonFX(IDConstants.IntakeMotorID);
    public TalonFX m_intakeFoldMotor = new TalonFX(IDConstants.IntakeFoldMotorID);
    static BeamBreak m_intakebeamBreak = new BeamBreak(0); // Is in robot TODO: Find channel id and remove fakeBeamBreak
    Trigger m_fakeBeamBreak = ControllerConstants.m_driveJoystick.button(4);
    public String m_state = "empty";
    NetworkTable m_table;
    static BeamBreak m_outtakeBeamBreak; // Checks to see if can score
    static BeamBreak m_elevatorBeamBreak; // Make sure coral doesn't block elevator

    public IntakeSubsystem() {
        m_table = NetworkTableInstance.getDefault().getTable("IntakeSubsystem");
        m_outtakeBeamBreak = new BeamBreak(1); // TODO: find the real channel IDs
        m_elevatorBeamBreak = new BeamBreak(2);
    }

    public void doIntaking() {
        if (isLoaded()) {
            m_intakeMotor.set(0);
            m_state = "loaded";
        }
        m_intakeMotor.set(IntakeConstants.intakeSpeed);
        m_state = "intaking";
    }

    public void stopIntake() {
        m_intakeMotor.set(0);
        if (m_fakeBeamBreak.getAsBoolean()) {// TODO: use beamBroken method once available
            m_state = "shoot"; // TODO: Should this be outtaking?
        }
        // LED (on intake motor?) red
        // LED (on shoot motor?) green
    }

    public void shoot() {
        // TODO: use beamBroken method once available
        if (m_state == "shoot" && m_fakeBeamBreak.getAsBoolean()) {
            m_state = "empty";
            // LED (on intake motor?) green
            // LED (on shoot motor?) red
        }
    }

    public void periodic() {
        m_table.putValue("state", NetworkTableValue.makeString(m_state));
    }

    public boolean haveCoral() {
        if (m_elevatorBeamBreak.beamBroken() || m_intakebeamBreak.beamBroken() || m_outtakeBeamBreak.beamBroken()) {
            return true;
        }
        return false;
    }

    public static boolean isLoaded() {
        if (m_outtakeBeamBreak.beamBroken() && !m_elevatorBeamBreak.beamBroken()) {
            return true;
        }
        return false;
    }
    public static boolean isEmpty() {
        if (!m_elevatorBeamBreak.beamBroken() && !m_intakebeamBreak.beamBroken() && !m_outtakeBeamBreak.beamBroken()) {
            return true;
        }
        return false;
    }
}
