package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IDConstants;

public class ElevatorSubsystem {
    TalonFX m_elevatorMotor1 = new TalonFX(IDConstants.ElevatorMotor1ID);
    TalonFX m_elevatorMotor2 = new TalonFX(IDConstants.ElevatorMotor2ID);
    public void elevatorUp() {
        // TODO: These are probably not the right speeds, should be fixed
        m_elevatorMotor1.set(0.2);
        m_elevatorMotor2.set(0.2);
    }
    public void elevatorDown() {
        // TODO: These are probably not the right speeds, should be fixed
        m_elevatorMotor1.set(-0.2);
        m_elevatorMotor2.set(-0.2);
        m_elevatorMotor1.setControl(new PositionDutyCycle(ElevatorConstants.ElevatorDownPosition));
        m_elevatorMotor2.setControl(new PositionDutyCycle(ElevatorConstants.ElevatorDownPosition));
        // m_elevatorMotor1.resetMotorEncoders();
    }
    public void resetMotorEncoders() {
        return;
        // TODO: write an actual function here or get rid of it if we don't use it
    }
    public void L4ButtonIsPressed() {
        m_elevatorMotor1.setControl(new PositionDutyCycle(ElevatorConstants.L4Position));
        m_elevatorMotor2.setControl(new PositionDutyCycle(ElevatorConstants.L4Position));
    }
    public void L3ButtonIsPressed() {
        m_elevatorMotor1.setControl(new PositionDutyCycle(ElevatorConstants.L3Position));
        m_elevatorMotor2.setControl(new PositionDutyCycle(ElevatorConstants.L3Position));
    }
}
