// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.DriveConstants;

public class SwerveModule {

    public final TalonFX m_driveMotor;
    private final TalonFX m_turningMotor;
    private final double m_absEncoderForward;
    public double m_turningOffset;
    public final DutyCycleEncoder m_absEncoder;
    String m_name;
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable driveTrainTable = inst.getTable("drive-train");

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, turning encoder,
     * absolute encoder corresponding to forward, and a name.
     *
     * @param driveMotorChannel   CAN ID for the drive motor.
     * @param turningMotorChannel CAN ID for the turning motor.
     * @param absEncoder          DIO port for absolute encoder
     * @param absEncoderForward   Absolute Encoder Value that points forward
     * @param name                Name of the swerve module
     * 
     */

    public SwerveModule(
            int driveMotorChannel,
            int turningMotorChannel, int absEncoder, double absEncoderForward, String name) {
        m_driveMotor = new TalonFX(driveMotorChannel, "CANivore");
        var slot0Configs = new Slot0Configs();
        new ClosedLoopRampsConfigs().withTorqueClosedLoopRampPeriod(DriveConstants.rampTimeTo300s);
        var currentConfigs = new TorqueCurrentConfigs().withPeakForwardTorqueCurrent(DriveConstants.currentMax)
                .withPeakReverseTorqueCurrent(DriveConstants.currentMin);

        slot0Configs.kV = DriveConstants.drivekF;
        slot0Configs.kP = DriveConstants.drivekP;
        slot0Configs.kI = DriveConstants.drivekI;
        slot0Configs.kD = DriveConstants.drivekD;
        m_driveMotor.getConfigurator().apply(new TalonFXConfiguration());
        m_driveMotor.getConfigurator().apply(slot0Configs);
        // m_driveMotor.getConfigurator().apply(rampConfigs);
        m_driveMotor.getConfigurator().apply(currentConfigs);

        m_driveMotor.setNeutralMode(NeutralModeValue.Coast);

        // BaseStatusSignal.setUpdateFrequencyForAll(50,m_driveMotor.getClosedLoopError(),
        // m_driveMotor.getClosedLoopDerivativeOutput(),m_driveMotor.getClosedLoopIntegratedOutput(),m_driveMotor.getClosedLoopProportionalOutput(),m_driveMotor.getClosedLoopFeedForward());

        m_turningMotor = new TalonFX(turningMotorChannel, "CANivore");
        TalonFXConfiguration turningConfig = new TalonFXConfiguration();
        turningConfig.Slot0.kP = DriveConstants.turningkP;
        turningConfig.Slot0.kI = DriveConstants.turningkI;
        turningConfig.Slot0.kD = DriveConstants.turningkD;
        turningConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_turningMotor.getConfigurator().apply(new TalonFXConfiguration());
        m_turningMotor.getConfigurator().apply(turningConfig);
        m_driveMotor.getConfigurator().apply(currentConfigs);
        m_turningMotor.setNeutralMode(NeutralModeValue.Brake);

        var rampConfigsTurning = new ClosedLoopRampsConfigs()
                .withDutyCycleClosedLoopRampPeriod(DriveConstants.rampTimeTo300s);
        var currentConfigsTurning = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(DriveConstants.currentLimit)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLowerLimit(DriveConstants.currentThreshold) // used to be SupplyCurrentThreshold
                .withSupplyCurrentLowerTime(DriveConstants.currentTimeThreshold); // used to be SupplyTimeThreshold

        m_turningMotor.getConfigurator().apply(currentConfigsTurning);
        m_turningMotor.getConfigurator().apply(rampConfigsTurning);

        m_absEncoder = new DutyCycleEncoder(absEncoder);
        m_turningOffset = 0.0;
        m_absEncoderForward = absEncoderForward;
        m_name = name;
    }

    public void resetOffset() {

        if (m_absEncoder.get() != 0) {
            double offset = m_absEncoderForward - m_absEncoder.get();
            m_turningOffset = m_turningMotor.getPosition().getValueAsDouble()
                    + offset * (DriveConstants.kEncoderResolution * DriveConstants.turningGearRatio);
        }

    }

    public double radiansToTicks(double radians) {
        return radians * ((DriveConstants.kEncoderResolution * DriveConstants.turningGearRatio) / (2 * Math.PI));
    }

    public double ticksToRadians(double ticks) {
        return ticks * ((2 * Math.PI) / (DriveConstants.kEncoderResolution * DriveConstants.turningGearRatio));
    }

    public double ticksToMeters(double ticks) {
        return ((ticks / DriveConstants.kEncoderResolution) * (2 * Math.PI * DriveConstants.kWheelRadius))
                / DriveConstants.driveGearRatio;
    }

    public double metersToTicks(double meters) {
        return (meters / (2 * Math.PI * DriveConstants.kWheelRadius)) * DriveConstants.kEncoderResolution
                * DriveConstants.driveGearRatio;

    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        double currentAngle = ticksToRadians(motorValueToTicks(m_turningMotor.getRotorPosition().getValueAsDouble()));
        currentAngle = MathUtil.inputModulus(currentAngle, desiredState.angle.getRadians() - Math.PI,
                desiredState.angle.getRadians() + Math.PI);
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(currentAngle));
        driveTrainTable.putValue(m_name + "turningOffset", NetworkTableValue.makeDouble(m_turningOffset));
        double currentPosition = m_turningMotor.getPosition().getValueAsDouble();
        double goalPosition = MathUtil.inputModulus(ticksToMotorValue(radiansToTicks(state.angle.getRadians())),
                currentPosition - Constants.DriveConstants.turningGearRatio / 2,
                currentPosition + Constants.DriveConstants.turningGearRatio / 2);
        m_turningMotor.setControl(new PositionDutyCycle(goalPosition));
        driveTrainTable.putValue(m_name + "turningSetpoint", NetworkTableValue.makeDouble(goalPosition));
        driveTrainTable.putValue(m_name + "driveSetpoint",
                NetworkTableValue.makeDouble(metersToTicks(state.speedMetersPerSecond)));

        m_driveMotor.setControl(new VelocityTorqueCurrentFOC(metersToTicks(state.speedMetersPerSecond)));

    }

    public double getModifiedAbsolute() {
        double absValue = m_absEncoder.get() - m_absEncoderForward;
        return absValue * 2 * Math.PI;
    }

    public double motorValueToTicks(double position) {
        return position - m_turningOffset;
    }

    public double ticksToMotorValue(double position) {
        return position + m_turningOffset;
    }

    public SwerveModulePosition getPosition() {
        // driveTrainTable.putValue(m_name + "currentAngle",
        // NetworkTableValue.makeDouble(ticksToRadians(motorValueToTicks(m_turningMotor.getRotorPosition().getValue()))));
        // driveTrainTable.putValue(m_name + "testAngle",
        // NetworkTableValue.makeDouble(getModifiedAbsolute()));
        if (m_absEncoder.get() != 0) {
            return new SwerveModulePosition(
                    ticksToMeters(m_driveMotor.getRotorPosition().getValueAsDouble()),
                    new Rotation2d(getModifiedAbsolute()));
        }
        return new SwerveModulePosition(
                ticksToMeters(m_driveMotor.getRotorPosition().getValueAsDouble()),
                new Rotation2d(
                        ticksToRadians(motorValueToTicks(m_turningMotor.getRotorPosition().getValueAsDouble()))));
    }

    public double getEncoderAngle() {
        return m_turningMotor.getRotorPosition().getValueAsDouble(); // do we want a double here?
    }

    public double turnPower() {
        return m_turningMotor.get();
    }

    public double drivePower() {
        return m_driveMotor.get();
    }

    public double driveSpeed() {
        return m_driveMotor.getVelocity().getValueAsDouble();
    }

    public void updateShuffleboard() {
        // driveTrainTable.putValue(m_name + " Drive Power",
        // NetworkTableValue.makeDouble(drivePower()));
        // driveTrainTable.putValue(m_name + " Drive Velocity",
        // NetworkTableValue.makeDouble(driveSpeed()));
        // driveTrainTable.putValue(m_name + " Turn Power",
        // NetworkTableValue.makeDouble(turnPower()));
        // driveTrainTable.putValue(m_name + " Turn Angle",
        // NetworkTableValue.makeDouble(getEncoderAngle()));
        driveTrainTable.putValue(m_name + " Abs Encoder",
                NetworkTableValue.makeDouble(m_absEncoder.get()));
        // driveTrainTable.putValue(m_name + " Turning kD Proportion",
        // NetworkTableValue.makeDouble(m_turningMotor.getClosedLoopDerivativeOutput().getValue()));
        // driveTrainTable.putValue(m_name + " Turning kP Proportion",
        // NetworkTableValue.makeDouble(m_turningMotor.getClosedLoopProportionalOutput().getValue()));
        // driveTrainTable.putValue(m_name + " Turning kI Proportion",
        // NetworkTableValue.makeDouble(m_turningMotor.getClosedLoopIntegratedOutput().getValue()));
        // driveTrainTable.putValue(m_name + " Turning kF Proportion",
        // NetworkTableValue.makeDouble(m_turningMotor.getClosedLoopFeedForward().getValue()));

        // driveTrainTable.putValue(m_name + " Driving kD Proportion",
        // NetworkTableValue.makeDouble(m_driveMotor.getClosedLoopDerivativeOutput().getValue()));
        // driveTrainTable.putValue(m_name + " Driving Error",
        // NetworkTableValue.makeDouble(m_driveMotor.getClosedLoopError().getValue()));
        // driveTrainTable.putValue(m_name + " Driving kP Proportion",
        // NetworkTableValue.makeDouble(m_driveMotor.getClosedLoopProportionalOutput().getValue()));
        // driveTrainTable.putValue(m_name + " Driving kI Proportion",
        // NetworkTableValue.makeDouble(m_driveMotor.getClosedLoopIntegratedOutput().getValue()));
        // driveTrainTable.putValue(m_name + " Driving kF Proportion",
        // NetworkTableValue.makeDouble(m_driveMotor.getClosedLoopFeedForward().getValue()));
    }
}
