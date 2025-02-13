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
    public double targetRotation = 0;
    public String elevatorPreset = "Empty";
    public double targetPosition = 0; // TODO: find real value
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
        // m_elevatorMotor1.setControl(new
        // PositionDutyCycle(ElevatorConstants.ElevatorDownPosition));
        // m_elevatorMotor2.setControl(new
        // PositionDutyCycle(ElevatorConstants.ElevatorDownPosition));
        // resetMotorEncoders();
    }

    public void resetMotorEncoders() {
        if (atBottom()) {
            m_offset = ElevatorConstants.bottomPosition - m_elevatorMotor1.getRotorPosition().getValueAsDouble();
            // climbMotorLeft.setSoftLimit(SoftLimitDirection.kForward, (float)
            // (ClimberConstants.topLeftPosition - m_offset));
            // climbMotorLeft.setSoftLimit(SoftLimitDirection.kReverse,
            // (float) (ClimberConstants.bottomPosition - m_offset));
        }
    }

    public void L4ButtonIsPressed() {
        targetRotation = Constants.ElevatorConstants.L4Position;
        elevatorPreset = "Elevator to L4";
    }

    public void L3ButtonIsPressed() {
        targetRotation = Constants.ElevatorConstants.L3Position;
        elevatorPreset = "Elevator to L3";
    }

    public void L2ButtonIsPressed() {
        targetRotation = Constants.ElevatorConstants.L2Position;
        elevatorPreset = "Elevator to L2";
    }

    public void L1ButtonIsPressed() {
        targetRotation = Constants.ElevatorConstants.L1Position;
        elevatorPreset = "Elevator to L1";
    }

    public void GoToL4() {
        // m_elevatorMotor1.setPosition(Constants.ElevatorConstants.L4Position +
        // m_offset);
    }

    public void GoToL3() {
        // m_elevatorMotor1.setPosition(Constants.ElevatorConstants.L3Position +
        // m_offset);
    }

    public void GoToL2() {
        // m_elevatorMotor1.setPosition(Constants.ElevatorConstants.L2Position +
        // m_offset);
    }

    public void GoToL1() {
        // m_elevatorMotor1.setPosition(Constants.ElevatorConstants.L1Position +
        // m_offset);
    }

    public void GoToTarget() {
        // m_elevatorMotor1.setPosition(Constants.ElevatorConstants.L1Position +
        // m_offset);
    }

    public void GoToElevatorDown() {
        // m_elevatorMotor1.setPosition(targetRotation + m_offset);
    }

    public boolean atBottom() {
        if (m_bottomLimit.getValue() == ReverseLimitValue.ClosedToGround) {
            return true;
        } else {
            return false;
        }
    }

    public Command SetPosition(double position) {
        return new InstantCommand(() -> m_elevatorMotor1.setPosition(position));
    }

    @Override
    public void periodic() {
        m_table.putValue("state", NetworkTableValue.makeString(elevatorPreset));
    }
}