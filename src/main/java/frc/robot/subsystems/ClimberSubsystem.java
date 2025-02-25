package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.commands.ClimberClimbCommand;

public class ClimberSubsystem extends SubsystemBase {
    TalonFX climbMotor = new TalonFX(IDConstants.climbMotorID);

    SparkMax m_pinnerMotor = new SparkMax(IDConstants.pinnerMotorID, MotorType.kBrushless);
    SparkMax m_releaseMotor = new SparkMax(IDConstants.releaseMotorID, MotorType.kBrushless);
    SparkClosedLoopController m_pinnerController = m_pinnerMotor.getClosedLoopController();
    SparkMaxConfig m_pinnerConfig = new SparkMaxConfig();

    NetworkTable m_climberTable;
    DutyCycleEncoder m_pinnerAbsEncoder;

    public ClimberSubsystem() {
        m_pinnerAbsEncoder = new DutyCycleEncoder(IDConstants.pinnerAbsEncoderID);
        m_climberTable = NetworkTableInstance.getDefault().getTable("ClimberSubsystem");
        m_pinnerConfig.closedLoop
                .p(ClimberConstants.pinnerkP)
                .i(ClimberConstants.pinnerkI)
                .d(ClimberConstants.pinnerkD)
                .outputRange(ClimberConstants.pinnerkMinOutput, ClimberConstants.pinnerkMaxOutPut);
        m_pinnerMotor.configure(m_pinnerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void climb() {
        climbMotor.set(0.2); // TODO: Find good speed
    }

    public void stopClimb() {
        climbMotor.set(0);
    }

    public void initializePinnerRelativeEncoder() {
        double position = (m_pinnerAbsEncoder.get() - ClimberConstants.pinnerVertical) * 60;
        m_pinnerMotor.getEncoder().setPosition(position);
    }

    public Command openPinner() {
        return new InstantCommand(
                () -> {
                    m_pinnerController.setReference(0,
                            // m_pinnerMotor.getEncoder().getPosition() +
                            // ClimberConstants.pinnerQuarterRotation,
                            ControlType.kPosition);
                },
                this);
    }

    public Command closePinner() {
        return new InstantCommand(
                () -> {
                    m_pinnerController.setReference(15,
                            // m_pinnerMotor.getEncoder().getPosition() +
                            // ClimberConstants.pinnerQuarterRotation,
                            ControlType.kPosition);
                },
                this);

    }

    @Override
    public void periodic() {
        m_climberTable.putValue("pinnerAbsEncoder", NetworkTableValue.makeDouble(m_pinnerAbsEncoder.get()));
        m_climberTable.putValue("pinnerRelativeEncoder",
                NetworkTableValue.makeDouble(m_pinnerMotor.getEncoder().getPosition()));
    }

}