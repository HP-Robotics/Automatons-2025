package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IDConstants;

public class ClimberSubsystem extends SubsystemBase {

    SparkMax m_releaseMotor = new SparkMax(IDConstants.releaseMotorID, MotorType.kBrushless);

    NetworkTable m_climberTable;

    TalonFX m_climbMotor = new TalonFX(IDConstants.ClimberMotorID, "CANivore");
    DutyCycleEncoder m_absEncoder;
    NetworkTable m_table;
    double m_offset = 0.0;
    Timer m_timer;

    public ClimberSubsystem() {
        m_climberTable = NetworkTableInstance.getDefault().getTable("ClimberSubsystem");

        m_table = NetworkTableInstance.getDefault().getTable("ClimberSubsystem");
        m_climbMotor.getConfigurator().apply(new TalonFXConfiguration());

        var slot0Configs = new Slot0Configs();
        slot0Configs.kV = ClimberConstants.climbModulekF;
        slot0Configs.kP = ClimberConstants.climbModulekP;
        slot0Configs.kI = ClimberConstants.climbModulekI;
        slot0Configs.kD = ClimberConstants.climbModulekD;
        m_climbMotor.getConfigurator().apply(slot0Configs);

        m_climbMotor.setNeutralMode(NeutralModeValue.Brake);
        // MotorOutputConfigs motorConfigs = new MotorOutputConfigs();

        // motorConfigs.PeakForwardDutyCycle = 0.1;
        // motorConfigs.PeakReverseDutyCycle = -0.1;
        // m_climbMotor.getConfigurator().apply(motorConfigs); // temporary

        m_absEncoder = new DutyCycleEncoder(6);

        m_timer = new Timer();
        m_timer.start();
    }

    public void setClimbMotorConfigs() {
        var climberSoftLimits = new SoftwareLimitSwitchConfigs();
        climberSoftLimits.ForwardSoftLimitThreshold = ClimberConstants.climberIn + m_offset;
        climberSoftLimits.ReverseSoftLimitThreshold = ClimberConstants.climberOut + m_offset;
        climberSoftLimits.ForwardSoftLimitEnable = true;
        climberSoftLimits.ReverseSoftLimitEnable = true;
        m_climbMotor.getConfigurator().apply(climberSoftLimits);
    }

    public Command Climb() {
        return new RunCommand(
                () -> m_climbMotor.setControl(new PositionDutyCycle(ClimberConstants.climberIn + m_offset)));
    }

    public Command StopClimb() {
        return new InstantCommand(() -> m_climbMotor.setControl(new NeutralOut()));
    }

    public Command RaiseClimber() {
        return new InstantCommand(
                () -> m_climbMotor.setControl(new PositionDutyCycle(ClimberConstants.climberOut + m_offset)));
    }

    public Command ResetClimber() {
        return new InstantCommand(
                () -> m_climbMotor.setControl(new PositionDutyCycle(ClimberConstants.climberReset + m_offset)));
    }



    public Optional<Double> getAbsEncoder() {

        return Optional.empty();
        // var encoderAbs = m_absEncoder.get();
        // if (Math.abs(encoderAbs) == 0 || Math.abs(encoderAbs) == 1) {
        //     return Optional.empty();
        // } else {
        //     return Optional.of(MathUtil.inputModulus(encoderAbs,
        //             ClimberConstants.climberStartAbsolute - 1 + ClimberConstants.encoderModulusTolerance,
        //             ClimberConstants.climberStartAbsolute + ClimberConstants.encoderModulusTolerance));
        // }
    }

    @Override
    public void periodic() {

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
        if (m_timer.hasElapsed(5)) {
            var encoderAbs = getAbsEncoder();
            if (encoderAbs.isPresent()) 
                if (m_offset == 0
                        || Math.abs(ClimberConstants.climberIn + m_offset - m_climbMotor.getRotorPosition()
                                .getValueAsDouble()) > ClimberConstants.offsetErrorThreshold) {
                    m_offset = (encoderAbs.get() - ClimberConstants.climberStartAbsolute)
                            * ClimberConstants.climberGearRatio
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
                }
            
            }
           // m_table.putValue("AbsEncoder", NetworkTableValue.makeDouble(m_absEncoder.get()));
            m_table.putValue("Relative encoder",
                    NetworkTableValue.makeDouble(m_climbMotor.getRotorPosition().getValueAsDouble()));
           // m_table.putValue("Offset", NetworkTableValue.makeDouble(m_offset));
           // m_table.putValue("Error",
                   // NetworkTableValue.makeDouble(m_climbMotor.getClosedLoopError().getValueAsDouble()));
            m_table.putValue("Down Setpoint",
                    NetworkTableValue.makeDouble(ClimberConstants.climberIn + m_offset));
        
    }
}