package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IDConstants;

public class ClimberSubsystem extends SubsystemBase {
    TalonFX climbMotor = new TalonFX(IDConstants.ClimbMotorID);

    public void climb() {
        climbMotor.set(0.2); // TODO: Find good speed
    }

    public void stopClimb() {
        climbMotor.set(0);
    }

}