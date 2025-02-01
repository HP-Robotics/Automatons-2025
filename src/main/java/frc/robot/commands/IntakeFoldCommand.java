package frc.robot.commands;

import org.ejml.equation.Sequence;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeFoldCommand {
    IntakeSubsystem m_intakeSubsystem;
    Timer m_timer = new Timer();

    public IntakeFoldCommand(IntakeSubsystem intakeSubsystem) {
        m_intakeSubsystem = intakeSubsystem;
    }

    public void initialize() {
        m_timer.start();
        if (Constants.SubsystemConstants.useIntake == true) {
            m_intakeSubsystem.m_intakeFoldMotor.set(Constants.ClimberConstants.foldSpeed);
        }

    }

    public void end() {
        m_intakeSubsystem.m_intakeFoldMotor.set(0);
    }

    public boolean isFinished() {
        if (m_timer.hasElapsed(Constants.ClimberConstants.foldRunTime)) {
            return true;
        } else {
            return false;
        }
    }
}