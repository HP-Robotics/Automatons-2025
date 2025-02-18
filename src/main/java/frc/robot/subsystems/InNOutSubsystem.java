package frc.robot.subsystems;

import frc.robot.BeamBreak;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.Constants.PortConstants;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class InNOutSubsystem extends SubsystemBase {
    TalonFX m_intakeMotor = new TalonFX(IDConstants.intakeMotorID);
    TalonFX m_outtakeMotor = new TalonFX(IDConstants.outtakeMotorID);
    public TalonFX m_intakeFoldMotor = new TalonFX(IDConstants.intakeFoldMotorID);
    public String m_state = "empty";
    NetworkTable m_table;
    // BeamBreak m_intakeBeamBreak;
    BeamBreak m_outtakeBeamBreak; // Checks to see if can score
    BeamBreak m_elevatorBeamBreak; // Make sure coral doesn't block elevator

    public InNOutSubsystem() {
        m_table = NetworkTableInstance.getDefault().getTable("InNOutSubsystem");
        // m_intakeBeamBreak = new BeamBreak(PortConstants.intakeBeamBreakID);
        m_outtakeBeamBreak = new BeamBreak(PortConstants.outtakeBeamBreakID); // TODO: find the real channel IDs
        m_elevatorBeamBreak = new BeamBreak(PortConstants.elevatorBeamBreakID);
    }

    public void runIntake() {
        m_intakeMotor.set(IntakeConstants.intakeSpeed);
    }

    public void runOuttake() {
        m_outtakeMotor.set(OuttakeConstants.outtakeSpeed);
    }

    public void loadOuttake() {
        m_outtakeMotor.set(OuttakeConstants.loadSpeed);
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
        // m_table.putValue("Intake Beam Broken",
        // NetworkTableValue.makeBoolean(m_intakeBeamBreak.beamBroken()));
        m_table.putValue("Outtake Beam Broken", NetworkTableValue.makeBoolean(m_outtakeBeamBreak.beamBroken()));
        m_table.putValue("Elevator Beam Broken", NetworkTableValue.makeBoolean(m_elevatorBeamBreak.beamBroken()));
    }

    public boolean intakeHasCoral() {
        return m_elevatorBeamBreak.beamBroken(); // || m_intakeBeamBreak.beamBroken();
    }

    public boolean isLoaded() {
        return m_outtakeBeamBreak.beamBroken() && !m_elevatorBeamBreak.beamBroken();
    }

    public boolean isEmpty() {
        return !m_elevatorBeamBreak.beamBroken()
                /* && !m_intakeBeamBreak.beamBroken() */ && !m_outtakeBeamBreak.beamBroken();
    }

    public Command IntakeCoral() { // make this timed with a state condition
        if (this.m_state == "intaking" || this.m_state == "empty")
            return new ParallelCommandGroup(
                    new StartEndCommand(this::runIntake, this::stopIntake),
                    new StartEndCommand(this::runOuttake, this::stopOuttake));
        else {
            return null;
        }
    }

    public Command OuttakeCoral() {
        return new StartEndCommand(this::runOuttake, this::stopOuttake);
    }
}
