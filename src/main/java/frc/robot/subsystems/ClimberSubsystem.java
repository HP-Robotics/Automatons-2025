package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IDConstants;

public class ClimberSubsystem extends SubsystemBase {
    TalonFX m_climbMotor = new TalonFX(IDConstants.ClimberMotorID);
    DutyCycleEncoder m_absEncoder;
    NetworkTable m_table;
    Double m_offset = 0.0;
    Timer m_timer;

    public ClimberSubsystem() {
        m_table = NetworkTableInstance.getDefault().getTable("ClimberSubsystem");
        m_climbMotor.getConfigurator().apply(new TalonFXConfiguration());

        var slot0Configs = new Slot0Configs();
        slot0Configs.kV = ClimberConstants.climbModulekF;
        slot0Configs.kP = ClimberConstants.climbModulekP;
        slot0Configs.kI = ClimberConstants.climbModulekI;
        slot0Configs.kD = ClimberConstants.climbModulekD;
        m_climbMotor.getConfigurator().apply(slot0Configs);

        m_climbMotor.setNeutralMode(NeutralModeValue.Brake);
        MotorOutputConfigs motorConfigs = new MotorOutputConfigs();

        // motorConfigs.PeakForwardDutyCycle = 0.1;
        // motorConfigs.PeakReverseDutyCycle = -0.1;
        // m_climbMotor.getConfigurator().apply(motorConfigs); // temporary

        m_absEncoder = new DutyCycleEncoder(6);

        m_timer = new Timer();
        m_timer.start();

    }

    public Command Climb() {
        return new InstantCommand(
                () -> m_climbMotor.setControl(new PositionDutyCycle(ClimberConstants.climberDownRelative + m_offset)));
    }

    public Command StopClimb() {
        return new InstantCommand(() -> m_climbMotor.set(0.0));
    }

    public Command ResetClimmber() {
        return new InstantCommand(
                () -> m_climbMotor.setControl(new PositionDutyCycle(ClimberConstants.climberUpRelative + m_offset)));
    }

    @Override
    public void periodic() {
        if (m_offset == 0.0) {
            if (m_timer.hasElapsed(5)) {
                var encoderAbs = m_absEncoder.get();
                if (Math.abs(encoderAbs) > 0 && Math.abs(encoderAbs) < 1) {
                    m_offset = (encoderAbs - ClimberConstants.climberUpAbsolute) * ClimberConstants.climberGearRatio
                            + m_climbMotor.getRotorPosition().getValueAsDouble();

                }
            }
        }

        m_table.putValue("AbsEncoder", NetworkTableValue.makeDouble(m_absEncoder.get()));
        m_table.putValue("Relative encoder",
                NetworkTableValue.makeDouble(m_climbMotor.getRotorPosition().getValueAsDouble()));
        m_table.putValue("Offset", NetworkTableValue.makeDouble(m_offset));

    }
}