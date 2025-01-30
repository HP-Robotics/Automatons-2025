package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IDConstants;

public class ElevatorSubsystem extends SubsystemBase {
    TalonFX m_elevatorMotor1 = new TalonFX(IDConstants.ElevatorMotor1ID);
    TalonFX m_elevatorMotor2 = new TalonFX(IDConstants.ElevatorMotor2ID);
    Slot0Configs m_PIDValues = new Slot0Configs();
    public ElevatorSubsystem() {
        m_PIDValues.kP = ElevatorConstants.kP;
        m_PIDValues.kI = ElevatorConstants.kI;
        m_PIDValues.kD = ElevatorConstants.kD;
        m_elevatorMotor1.getConfigurator().apply(m_PIDValues);
    }
    public void elevatorDown() {
        m_elevatorMotor1.setControl(new PositionDutyCycle(ElevatorConstants.ElevatorDownPosition));
        m_elevatorMotor2.setControl(new PositionDutyCycle(ElevatorConstants.ElevatorDownPosition));
        resetMotorEncoders();
    }
    public void resetMotorEncoders() {
        m_elevatorMotor1.getEncoder().setPosition(0);
        m_elevatorMotor2.getEncoder().setPosition(0);
    }
    public void L4ButtonIsPressed() {
        m_elevatorMotor1.setControl(new PositionDutyCycle(ElevatorConstants.L4Position));
        m_elevatorMotor2.setControl(new PositionDutyCycle(ElevatorConstants.L4Position));
    }
    public void L3ButtonIsPressed() {
        m_elevatorMotor1.setControl(new PositionDutyCycle(ElevatorConstants.L3Position));
        m_elevatorMotor2.setControl(new PositionDutyCycle(ElevatorConstants.L3Position));
    }
    public void ElevatorDownButtonIsPressed() {
        elevatorDown();
    }
    public Command GoToL4() {
        return new InstantCommand(this::L4ButtonIsPressed);
    }
    public Command GoToL3() {
        return new InstantCommand(this::L3ButtonIsPressed);
    }
    public Command ElevatorDown() {
        return new InstantCommand(this::ElevatorDownButtonIsPressed);
    }
}