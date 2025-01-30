package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberClimbCommand extends Command {
  ClimberSubsystem m_climbSubsystem;

  public ClimberClimbCommand(ClimberSubsystem climberSubsystem) {
    m_climbSubsystem = climberSubsystem;
  }

  @Override
  public void initialize() {
    m_climbSubsystem.climb();
  }

  @Override
  public void end(boolean interrupted) {
    m_climbSubsystem.stopClimb();
  }
}