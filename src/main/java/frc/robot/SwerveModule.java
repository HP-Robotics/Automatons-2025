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
        public double m_relativeTurningOffset;
        public final DutyCycleEncoder m_absEncoder;
        String m_name;
        NetworkTableInstance m_inst = NetworkTableInstance.getDefault();
        NetworkTable m_driveTrainTable = m_inst.getTable("drive-train");

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
                // resets motor to factory default
                m_driveMotor.getConfigurator().apply(new TalonFXConfiguration());

                var slot0Configs = new Slot0Configs();
                slot0Configs.kV = DriveConstants.driveModulekF;
                slot0Configs.kP = DriveConstants.driveModulekP;
                slot0Configs.kI = DriveConstants.driveModulekI;
                slot0Configs.kD = DriveConstants.driveModulekD;
                m_driveMotor.getConfigurator().apply(slot0Configs);

                if (DriveConstants.useDriveRamp) {
                        var rampConfigs = new ClosedLoopRampsConfigs()
                                        .withTorqueClosedLoopRampPeriod(DriveConstants.driveRampTimeTo300s);
                        m_driveMotor.getConfigurator().apply(rampConfigs);
                }

                var currentConfigs = new TorqueCurrentConfigs()
                                .withPeakForwardTorqueCurrent(DriveConstants.driveCurrentMax)
                                .withPeakReverseTorqueCurrent(DriveConstants.driveCurrentMin);
                m_driveMotor.getConfigurator().apply(currentConfigs);

                m_driveMotor.setNeutralMode(NeutralModeValue.Coast);

                m_driveMotor.getRotorPosition().setUpdateFrequency(DriveConstants.odometryUpdateFrequency);
                // BaseStatusSignal.setUpdateFrequencyForAll(50,m_driveMotor.getClosedLoopError(),

                m_turningMotor = new TalonFX(turningMotorChannel, "CANivore");
                m_turningMotor.getConfigurator().apply(new TalonFXConfiguration());

                TalonFXConfiguration turningConfig = new TalonFXConfiguration();
                turningConfig.Slot0.kP = DriveConstants.turningModulekP;
                turningConfig.Slot0.kI = DriveConstants.turningModulekI;
                turningConfig.Slot0.kD = DriveConstants.turningModulekD;
                turningConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                m_turningMotor.getConfigurator().apply(turningConfig);
                m_turningMotor.setNeutralMode(NeutralModeValue.Brake);
                // TODO: consider deleting the current limit and ramp config
                var rampConfigsTurning = new ClosedLoopRampsConfigs()
                                .withDutyCycleClosedLoopRampPeriod(DriveConstants.turningRampTimeTo300s);
                var currentConfigsTurning = new CurrentLimitsConfigs()
                                .withSupplyCurrentLimit(DriveConstants.turnCurrentLimit)
                                .withSupplyCurrentLimitEnable(true)
                                .withSupplyCurrentLowerLimit(DriveConstants.turningCurrentThreshold)
                                // used to be SupplyCurrentThreshold
                                .withSupplyCurrentLowerTime(DriveConstants.turningCurrentTimeThreshold);
                // used to be SupplyTimeThreshold

                m_turningMotor.getConfigurator().apply(currentConfigsTurning);
                m_turningMotor.getConfigurator().apply(rampConfigsTurning);

                m_absEncoder = new DutyCycleEncoder(absEncoder);
                m_absEncoder.setInverted(false);
                m_relativeTurningOffset = 0.0;
                m_absEncoderForward = absEncoderForward;
                m_name = name;
        }

        public void resetOffset() {
                if (m_absEncoder.get() != 0) {
                        double offset = m_absEncoderForward - m_absEncoder.get();
                        m_relativeTurningOffset = m_turningMotor.getPosition().getValueAsDouble()
                                        + offset * (DriveConstants.kEncoderResolution
                                                        * DriveConstants.turningGearRatio);
                        m_driveTrainTable.putValue(m_name + "turning motor at startup",
                                        NetworkTableValue.makeDouble(m_turningMotor.getPosition().getValueAsDouble()));
                        m_driveTrainTable.putValue(m_name + "abs encoder at startup",
                                        NetworkTableValue.makeDouble(m_absEncoder.get()));
                        m_driveTrainTable.putValue(m_name + " offset", NetworkTableValue.makeDouble(offset));
                }

        }

        public double radiansToTicks(double radians) {
                return radians * ((DriveConstants.kEncoderResolution * DriveConstants.turningGearRatio)
                                / (2 * Math.PI));
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
                // State or radians means that it's in radians
                // Relative or setpoint means that it's in kraken/falcon units
                // Abs means that it's absolute encoder units

                m_driveTrainTable.putValue(m_name + " desiredState",
                                NetworkTableValue.makeDouble(desiredState.angle.getRadians()));
                double modifiedCurrentAngle = MathUtil.inputModulus(getRadiansAngle(),
                                desiredState.angle.getRadians() - Math.PI,
                                desiredState.angle.getRadians() + Math.PI); // Limit current robot angle within pi of
                                                                            // desired angle

                desiredState.optimize(new Rotation2d(modifiedCurrentAngle));
                // Optimize the reference state to avoid spinning further than 90 degrees

                double currentRelativeAngle = getEncoderAngle();
                double desiredSetpoint = MathUtil.inputModulus(
                                ticksToMotorValue(radiansToTicks(desiredState.angle.getRadians())),
                                currentRelativeAngle - Constants.DriveConstants.turningGearRatio / 2,
                                currentRelativeAngle + Constants.DriveConstants.turningGearRatio / 2);
                // Compute a desired setpoint from the relative encoder and make it less than
                // half of a rotation

                m_turningMotor.setControl(new PositionDutyCycle(desiredSetpoint));
                m_driveMotor.setControl(new VelocityTorqueCurrentFOC(metersToTicks(desiredState.speedMetersPerSecond)));

                // TODO: Comment out the shuffleboard values we don't need
                m_driveTrainTable.putValue(m_name + " optimizedDesiredState",
                                NetworkTableValue.makeDouble(desiredState.angle.getRadians()));
                m_driveTrainTable.putValue(m_name + " relativeTurningOffset",
                                NetworkTableValue.makeDouble(m_relativeTurningOffset));
                m_driveTrainTable.putValue(m_name + " modifiedCurrentAngle",
                                NetworkTableValue.makeDouble(modifiedCurrentAngle));
                m_driveTrainTable.putValue(m_name + " desiredSetpoint", NetworkTableValue.makeDouble(desiredSetpoint));
                m_driveTrainTable.putValue(m_name + " speedSetpoint",
                                NetworkTableValue.makeDouble(metersToTicks(desiredState.speedMetersPerSecond)));
        }

        public double getModifiedAbsolute() {
                double absValue = m_absEncoder.get() - m_absEncoderForward;
                return absValue * 2 * Math.PI;
        }

        public double motorValueToTicks(double position) {
                return position - m_relativeTurningOffset;
        }

        public double ticksToMotorValue(double position) {
                return position + m_relativeTurningOffset;
        }

        public double getRadiansAngle() {
                return ticksToRadians(motorValueToTicks(getEncoderAngle()));
        }

        public SwerveModulePosition getPosition() {
                if (m_absEncoder.get() != 0) {
                        return new SwerveModulePosition(
                                        ticksToMeters(m_driveMotor.getRotorPosition().getValueAsDouble()),
                                        new Rotation2d(getModifiedAbsolute()));
                }
                return new SwerveModulePosition(
                                ticksToMeters(m_driveMotor.getRotorPosition().getValueAsDouble()),
                                new Rotation2d(getRadiansAngle()));
        }

        public double getEncoderAngle() {
                return m_turningMotor.getRotorPosition().getValueAsDouble();
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
                m_driveTrainTable.putValue(m_name + " Relative Encoder Angle",
                                NetworkTableValue.makeDouble(getEncoderAngle()));
                m_driveTrainTable.putValue(m_name + " Abs Encoder",
                                NetworkTableValue.makeDouble(m_absEncoder.get()));
                m_driveTrainTable.putValue(m_name + " Abs Encoder Radians",
                                NetworkTableValue.makeDouble(getModifiedAbsolute()));
                m_driveTrainTable.putValue(m_name + " Relative Encoder Radians",
                                NetworkTableValue.makeDouble(getRadiansAngle()));
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