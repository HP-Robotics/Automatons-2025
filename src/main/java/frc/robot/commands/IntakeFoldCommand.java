package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.InNOutSubsystem;

public class IntakeFoldCommand extends Command {
    InNOutSubsystem m_inNOutSubsystem;

    public IntakeFoldCommand(InNOutSubsystem intakeSubsystem) {
        m_inNOutSubsystem = intakeSubsystem;
    }

    public void initialize() {
        if (Constants.SubsystemConstants.useIntake) {
            m_inNOutSubsystem.m_intakeFoldMotor.set(Constants.ClimberConstants.foldSpeed);
        }
    }

    public void end(boolean interrupted) {
        m_inNOutSubsystem.m_intakeFoldMotor.set(0);
    }

    public boolean isFinished() {
        return false;
    }
}