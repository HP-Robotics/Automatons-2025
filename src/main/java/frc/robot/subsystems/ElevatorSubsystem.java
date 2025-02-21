package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IDConstants;

public class ElevatorSubsystem extends SubsystemBase {
    TalonFX m_elevatorMotor1 = new TalonFX(IDConstants.ElevatorMotor1ID);
    TalonFX m_elevatorMotor2 = new TalonFX(IDConstants.ElevatorMotor2ID);
    Slot0Configs m_PIDValues = new Slot0Configs();
    public double m_targetRotation = 0;
    public String m_elevatorPreset = "Empty";
    public double m_targetPosition = 0; // TODO: find real value
    public double m_offset = 0;
    StatusSignal m_bottomLimit = m_elevatorMotor1.getReverseLimit();
    NetworkTable m_table;

    public ElevatorSubsystem() {
        m_table = NetworkTableInstance.getDefault().getTable("ElevatorSubsystem");

        TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
        elevatorConfig.Slot0.kP = ElevatorConstants.kP;
        elevatorConfig.Slot0.kI = ElevatorConstants.kI;
        elevatorConfig.Slot0.kD = ElevatorConstants.kD;
        elevatorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_elevatorMotor1.getConfigurator().apply(elevatorConfig);
        m_elevatorMotor2.getConfigurator().apply(elevatorConfig);
        m_elevatorMotor2.setControl(new Follower(m_elevatorMotor1.getDeviceID(), true));

    }

    public void elevatorDown() {
        m_elevatorMotor1.setControl(new
        PositionDutyCycle(ElevatorConstants.elevatorDownPosition));
        resetMotorEncoders();
    }

    public void resetMotorEncoders() {
        if (atBottom()) {
            m_offset = ElevatorConstants.bottomPosition - m_elevatorMotor1.getRotorPosition().getValueAsDouble();
            // climbMotorLeft.setSoftLimit(SoftLimitDirection.kForward, (float)
            // (ClimberConstants.topLeftPosition - m_offset));
            // climbMotorLeft.setSoftLimit(SoftLimitDirection.kReverse,
            // (float) (ClimberConstants.bottomPosition - m_offset));
            // TODO: find these numbers
        }
    }

    public void L4ButtonIsPressed() {
        m_targetRotation = Constants.ElevatorConstants.L4Position;
        m_elevatorPreset = "Elevator to L4";
    }

    public void L3ButtonIsPressed() {
        m_targetRotation = Constants.ElevatorConstants.L3Position;
        m_elevatorPreset = "Elevator to L3";
    }

    public void L2ButtonIsPressed() {
        m_targetRotation = Constants.ElevatorConstants.L2Position;
        m_elevatorPreset = "Elevator to L2";
    }

    public void L1ButtonIsPressed() {
        m_targetRotation = Constants.ElevatorConstants.L1Position;
        m_elevatorPreset = "Elevator to L1";
    }

    public void goToL4() {
        m_elevatorMotor1.setPosition(ElevatorConstants.L4Position + m_offset);
    }

    public void goToL3() {
        m_elevatorMotor1.setPosition(ElevatorConstants.L3Position + m_offset);
    }

    public void goToL2() {
        m_elevatorMotor1.setPosition(ElevatorConstants.L2Position + m_offset);
    }

    public void goToL1() {
        m_elevatorMotor1.setPosition(ElevatorConstants.L1Position + m_offset);
    }

    public void goToTarget() {
        m_elevatorMotor1.setPosition(m_targetRotation + m_offset);
    }

    public void goToIntake() {
        m_elevatorMotor1.setPosition(ElevatorConstants.elevatorDownPosition + m_offset);
    }

    public boolean atBottom() {
        return (m_bottomLimit.getValue() == ReverseLimitValue.ClosedToGround);
    }

    public Command SetPosition(double position) {
        return new InstantCommand(() -> m_elevatorMotor1.setPosition(position + m_offset));
    }

    @Override
    public void periodic() {
        m_table.putValue("state", NetworkTableValue.makeString(m_elevatorPreset));
    }
}