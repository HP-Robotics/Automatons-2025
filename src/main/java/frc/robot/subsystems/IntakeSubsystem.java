package frc.robot.subsystems;

import frc.robot.BeamBreak;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.OperatorConstants;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class IntakeSubsystem extends SubsystemBase {
    TalonFX m_intakeMotor = new TalonFX(IDConstants.IntakeMotorID);
    public TalonFX m_intakeFoldMotor = new TalonFX(IDConstants.IntakeFoldMotorID);
    BeamBreak m_beamBreak = new BeamBreak(0);// TODO: Find channel id and remove fakeBeamBreak
    Trigger m_fakeBeamBreak = new CommandXboxController(OperatorConstants.kDriverControllerPort).button(4);
    public String m_state = "empty";
    NetworkTable m_table;

    public IntakeSubsystem() {
        m_table = NetworkTableInstance.getDefault().getTable("IntakeSubsystem");
    }

    public void startIntake() {
        m_intakeMotor.set(0.2);
        m_state = "intaking";
    }

    public void stopIntake() {
        m_intakeMotor.set(0);
        if (m_fakeBeamBreak.getAsBoolean()) {// TODO: use beamBroken method once available
            m_state = "shoot"; // TODO: Should this be outtaking?
        } else {
            m_state = "empty";
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
}
