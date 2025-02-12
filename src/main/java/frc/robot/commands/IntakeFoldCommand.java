package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.InNOutSubsystem;

public class IntakeFoldCommand extends Command {
    InNOutSubsystem m_intakeSubsystem;

    public IntakeFoldCommand(InNOutSubsystem intakeSubsystem) {
        m_intakeSubsystem = intakeSubsystem;
    }

    public void initialize() {
        if (Constants.SubsystemConstants.useIntake) {
            m_intakeSubsystem.m_intakeFoldMotor.set(Constants.ClimberConstants.foldSpeed);
        }
    }

    public void end(boolean interrupted) {
        m_intakeSubsystem.m_intakeFoldMotor.set(0);
    }

    public boolean isFinished() {
        return false;
    }
}