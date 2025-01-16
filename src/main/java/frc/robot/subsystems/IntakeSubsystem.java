package frc.robot.subsystems;
import frc.robot.subsystems.DriveSubsystem;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
    TalonFX intake_motor = new TalonFX(0);//TO-DO: Find divice id
    String m_state = "empty";
    public IntakeSubsystem() {

    }
    public void StopIntake(){
        intake_motor.set(0);
        m_state = "standby";
    }
}
