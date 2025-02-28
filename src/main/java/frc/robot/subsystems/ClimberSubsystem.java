package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IDConstants;

public class ClimberSubsystem extends SubsystemBase {

    SparkMax m_pinnerMotor = new SparkMax(IDConstants.pinnerMotorID, MotorType.kBrushless);
    SparkMax m_releaseMotor = new SparkMax(IDConstants.releaseMotorID, MotorType.kBrushless);
    SparkClosedLoopController m_pinnerController = m_pinnerMotor.getClosedLoopController();
    SparkMaxConfig m_pinnerConfig = new SparkMaxConfig();

    NetworkTable m_climberTable;
    DutyCycleEncoder m_pinnerAbsEncoder;

    TalonFX m_climbMotor = new TalonFX(IDConstants.ClimberMotorID);
    DutyCycleEncoder m_absEncoder;
    NetworkTable m_table;
    Double m_offset = 0.0;
    Timer m_timer;

    public ClimberSubsystem() {
        m_pinnerAbsEncoder = new DutyCycleEncoder(IDConstants.pinnerAbsEncoderID);
        m_climberTable = NetworkTableInstance.getDefault().getTable("ClimberSubsystem");
        m_pinnerConfig.closedLoop
                .p(ClimberConstants.pinnerkP)
                .i(ClimberConstants.pinnerkI)
                .d(ClimberConstants.pinnerkD)
                .outputRange(ClimberConstants.pinnerkMinOutput, ClimberConstants.pinnerkMaxOutPut);
        m_pinnerMotor.configure(m_pinnerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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

    public void setClimbMotorConfigs() {
        var climberSoftLimits = new SoftwareLimitSwitchConfigs();
        climberSoftLimits.ForwardSoftLimitThreshold = ClimberConstants.climberDownRelative + m_offset;
        climberSoftLimits.ReverseSoftLimitThreshold = ClimberConstants.climberUpRelative + m_offset;
        climberSoftLimits.ForwardSoftLimitEnable = true;
        climberSoftLimits.ReverseSoftLimitEnable = true;
        m_climbMotor.getConfigurator().apply(climberSoftLimits);
    }

    public Command Climb() {
        return new InstantCommand(
                () -> m_climbMotor.setControl(new PositionDutyCycle(ClimberConstants.climberDownRelative + m_offset)));
    }

    public void initializePinnerRelativeEncoder() {
        double position = (m_pinnerAbsEncoder.get() - ClimberConstants.pinnerVertical)
                * ClimberConstants.pinnerGearRatio;
        m_pinnerMotor.getEncoder().setPosition(-position);
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
                    m_pinnerController.setReference(ClimberConstants.pinnerGearRatio / 4.0,
                            // m_pinnerMotor.getEncoder().getPosition() +
                            // ClimberConstants.pinnerQuarterRotation,
                            ControlType.kPosition);
                },
                this);
    }

    public Command StopClimb() {
        return new InstantCommand(() -> m_climbMotor.setControl(new NeutralOut()));
    }

    public Command ResetClimber() {
        return new InstantCommand(
                () -> m_climbMotor.setControl(new PositionDutyCycle(ClimberConstants.climberUpRelative + m_offset)));
    }

    @Override
    public void periodic() {
        m_climberTable.putValue("pinnerAbsEncoder", NetworkTableValue.makeDouble(m_pinnerAbsEncoder.get()));
        m_climberTable.putValue("pinnerRelativeEncoder",
                NetworkTableValue.makeDouble(m_pinnerMotor.getEncoder().getPosition()));

        /*
         * We have observed the absolute encoder reading 1.0 immediately on program
         * start.
         * Last year, we observed that the absolute encoder reading was not stable until
         * some time after boot. We noticed the encoder reading incorrectly early this
         * year
         * as well. So we have logic to wait 5 seconds before using the absolute
         * encoder,
         * and only if it is reading something sensible.
         */
        if (m_offset == 0.0) {
            if (m_timer.hasElapsed(5)) {
                var encoderAbs = m_absEncoder.get();
                if (Math.abs(encoderAbs) > 0 && Math.abs(encoderAbs) < 1) {
                    m_offset = (encoderAbs - ClimberConstants.climberUpAbsolute) * ClimberConstants.climberGearRatio
                            /*
                             * Note that the logic here would more naturally be expressed a subtracting
                             * the current relative encoder. That is, what we do is compute where we think
                             * the relative encoder should be, given where the absolute encoder is. Then, it
                             * would be natural to subtract the current value of the relative encoder.
                             * However, because the absolute encoder and relative encoder go opposite
                             * directions, we need
                             * one more negative sign. (To be clear, the absolute
                             * increases counter clockwise, or up, and relative increases clockwise, or
                             * 'down')
                             */
                            + m_climbMotor.getRotorPosition().getValueAsDouble();
                    setClimbMotorConfigs();
                }
                encoderAbs = m_pinnerAbsEncoder.get();
                if (Math.abs(encoderAbs) > 0 && Math.abs(encoderAbs) < 1) {
                    initializePinnerRelativeEncoder();
                }
            }
        }

        m_table.putValue("AbsEncoder", NetworkTableValue.makeDouble(m_absEncoder.get()));
        m_table.putValue("Relative encoder",
                NetworkTableValue.makeDouble(m_climbMotor.getRotorPosition().getValueAsDouble()));
        m_table.putValue("Offset", NetworkTableValue.makeDouble(m_offset));

    }
}