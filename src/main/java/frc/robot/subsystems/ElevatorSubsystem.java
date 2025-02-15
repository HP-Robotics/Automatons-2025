package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IDConstants;

public class ElevatorSubsystem extends SubsystemBase {
    public TalonFX m_elevatorMotor1 = new TalonFX(IDConstants.ElevatorMotor1ID);
    TalonFX m_elevatorMotor2 = new TalonFX(IDConstants.ElevatorMotor2ID);
    Slot0Configs m_PIDValues = new Slot0Configs();
    public double m_targetRotation = 0;
    public String elevatorPreset = "Empty";
    public double m_offset = 0;
    StatusSignal<ReverseLimitValue> m_bottomLimit = m_elevatorMotor1.getReverseLimit();
    NetworkTable m_table;
    TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
    final DoubleSubscriber kPSub;
    final DoubleSubscriber kISub;
    final DoubleSubscriber kDSub;
    final DoubleSubscriber kGSub;
    final DoubleSubscriber setpointSub;
    final DoubleSubscriber kASub;
    final DoubleSubscriber kSSub;
    final DoubleSubscriber kVSub;

    public ElevatorSubsystem() {
        m_table = NetworkTableInstance.getDefault().getTable("ElevatorSubsystem");

        elevatorConfig.Slot0.kP = ElevatorConstants.kP;
        elevatorConfig.Slot0.kI = ElevatorConstants.kI;
        elevatorConfig.Slot0.kD = ElevatorConstants.kD;
        elevatorConfig.Slot0.kS = ElevatorConstants.kS;
        elevatorConfig.Slot0.kV = ElevatorConstants.kV;
        elevatorConfig.Slot0.kA = ElevatorConstants.kA;
        elevatorConfig.Slot0.kG = ElevatorConstants.kG;
        elevatorConfig.HardwareLimitSwitch.ReverseLimitEnable = true;
        elevatorConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0;
        elevatorConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        elevatorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        elevatorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        elevatorConfig.Slot1.kP = ElevatorConstants.kP;
        elevatorConfig.Slot1.kI = ElevatorConstants.kI;
        elevatorConfig.Slot1.kD = ElevatorConstants.kD;
        elevatorConfig.Slot1.kS = ElevatorConstants.kS;
        elevatorConfig.Slot1.kV = 0.0;
        elevatorConfig.Slot1.kA = ElevatorConstants.kA;
        elevatorConfig.Slot1.kG = 0.0; // -ElevatorConstants.kG;

        // set Motion Magic settings
        elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.motionMagicCruiseVelocity; // Target
                                                                                                            // cruise
                                                                                                            // velocity
                                                                                                            // of 80 rps
        elevatorConfig.MotionMagic.MotionMagicAcceleration = ElevatorConstants.motionMagicAcceleration; // Target
                                                                                                        // acceleration
                                                                                                        // of 160 rps/s
                                                                                                        // (0.5 seconds)
        elevatorConfig.MotionMagic.MotionMagicJerk = ElevatorConstants.motionMagicJerk; // Target jerk of 1600 rps/s/s
                                                                                        // (0.1 seconds)

        m_elevatorMotor1.getConfigurator().apply(elevatorConfig);
        m_elevatorMotor2.getConfigurator().apply(elevatorConfig);
        m_elevatorMotor2.setControl(new Follower(m_elevatorMotor1.getDeviceID(), true));

        m_table.putValue("kP", NetworkTableValue.makeDouble(elevatorConfig.Slot0.kP));
        m_table.putValue("kI", NetworkTableValue.makeDouble(elevatorConfig.Slot0.kI));
        m_table.putValue("kD", NetworkTableValue.makeDouble(elevatorConfig.Slot0.kD));
        m_table.putValue("setpoint", NetworkTableValue.makeDouble(0));
        m_table.putValue("kG", NetworkTableValue.makeDouble(ElevatorConstants.kG));
        m_table.putValue("kA", NetworkTableValue.makeDouble(ElevatorConstants.kA));
        m_table.putValue("kS", NetworkTableValue.makeDouble(ElevatorConstants.kS));
        m_table.putValue("kV", NetworkTableValue.makeDouble(ElevatorConstants.kV));

        kPSub = m_table.getDoubleTopic("kP").subscribe(ElevatorConstants.kP);
        kISub = m_table.getDoubleTopic("kI").subscribe(ElevatorConstants.kI);
        kDSub = m_table.getDoubleTopic("kD").subscribe(ElevatorConstants.kD);
        kGSub = m_table.getDoubleTopic("kG").subscribe(ElevatorConstants.kG);
        kASub = m_table.getDoubleTopic("kA").subscribe(ElevatorConstants.kA);
        kVSub = m_table.getDoubleTopic("kV").subscribe(ElevatorConstants.kV);
        kSSub = m_table.getDoubleTopic("kS").subscribe(ElevatorConstants.kS);
        setpointSub = m_table.getDoubleTopic("setpoint").subscribe(0);

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
        m_targetRotation = Constants.ElevatorConstants.L4Position;
        elevatorPreset = "Elevator to L4";
    }

    public void L3ButtonIsPressed() {
        m_targetRotation = Constants.ElevatorConstants.L3Position;
        elevatorPreset = "Elevator to L3";
    }

    public void L2ButtonIsPressed() {
        m_targetRotation = Constants.ElevatorConstants.L2Position;
        elevatorPreset = "Elevator to L2";
    }

    public void L1ButtonIsPressed() {
        m_targetRotation = Constants.ElevatorConstants.L1Position;
        elevatorPreset = "Elevator to L1";
    }

    public void GoToL4() {
        goToPosition(Constants.ElevatorConstants.L4Position +
                m_offset);
    }

    public void GoToL3() {
        goToPosition(Constants.ElevatorConstants.L3Position +
                m_offset);
    }

    public void GoToL2() {
        goToPosition(Constants.ElevatorConstants.L2Position +
                m_offset);
    }

    public void GoToL1() {
        goToPosition(Constants.ElevatorConstants.L1Position +
                m_offset);
    }

    public void GoToTarget() {
        // m_elevatorMotor1.setPosition(m_targetRotation); // + m_offset
        // create a Motion Magic request, voltage output
        var request = new PositionVoltage(0).withPosition(m_targetRotation);
        if (m_targetRotation >= m_elevatorMotor1.getRotorPosition().getValueAsDouble()) {
            request.Slot = 0;
            m_elevatorMotor1.setControl(request);
            m_table.putValue("Slot", NetworkTableValue.makeInteger(m_elevatorMotor1.getClosedLoopSlot().getValue()));
        } else {
            request.Slot = 1;
            m_elevatorMotor1.setControl(request);
            m_table.putValue("Slot", NetworkTableValue.makeInteger(m_elevatorMotor1.getClosedLoopSlot().getValue()));
        }
    }

    public void goToPosition(double position) {
        var request = new PositionVoltage(0).withPosition(position);
        if (position >= m_elevatorMotor1.getRotorPosition().getValueAsDouble()) {
            request.Slot = 0;
            m_elevatorMotor1.setControl(request);
            m_table.putValue("Slot", NetworkTableValue.makeInteger(m_elevatorMotor1.getClosedLoopSlot().getValue()));
        } else {
            request.Slot = 1;
            m_elevatorMotor1.setControl(request);
            m_table.putValue("Slot", NetworkTableValue.makeInteger(m_elevatorMotor1.getClosedLoopSlot().getValue()));
        }
    }

    public void GoToElevatorDown() {
        goToPosition(ElevatorConstants.elevatorDownPosition + m_offset);
    }

    public boolean atBottom() {
        if (m_bottomLimit.getValue() == ReverseLimitValue.ClosedToGround) {
            return true;
        } else {
            return false;
        }
    }

    // are we there yet?
    public boolean atPosition() {
        return (Math.abs(m_elevatorMotor1.getClosedLoopError().getValueAsDouble()) <= ElevatorConstants.errorTolerance);
    }

    public boolean atDownPosition() {
        return (Math.abs(m_elevatorMotor1.getPosition().getValueAsDouble()
                - ElevatorConstants.elevatorDownPosition - m_offset) <= ElevatorConstants.errorTolerance
                && atPosition());
    }

    public Command SetPosition(double position) {
        return new InstantCommand(() -> goToPosition(position));
    }

    @Override
    public void periodic() {
        boolean doUpdate = false;
        for (double iterVal : kPSub.readQueueValues()) {
            elevatorConfig.Slot0.kP = iterVal;
            doUpdate = true;
        }
        for (double iterVal : kISub.readQueueValues()) {
            elevatorConfig.Slot0.kI = iterVal;
            doUpdate = true;
        }
        for (double iterVal : kDSub.readQueueValues()) {
            elevatorConfig.Slot0.kD = iterVal;
            doUpdate = true;
        }
        for (double iterVal : kASub.readQueueValues()) {
            elevatorConfig.Slot0.kA = iterVal;
            doUpdate = true;
        }
        for (double iterVal : kGSub.readQueueValues()) {
            elevatorConfig.Slot0.kG = iterVal;
            doUpdate = true;
        }
        for (double iterVal : kVSub.readQueueValues()) {
            elevatorConfig.Slot0.kV = iterVal;
            doUpdate = true;
        }
        for (double iterVal : kSSub.readQueueValues()) {
            elevatorConfig.Slot0.kS = iterVal;
            doUpdate = true;
        }
        for (double iterVal : setpointSub.readQueueValues()) {
            m_targetRotation = iterVal;
            doUpdate = true;
        }
        if (doUpdate) {
            m_elevatorMotor1.getConfigurator().apply(elevatorConfig);
            m_elevatorMotor2.getConfigurator().apply(elevatorConfig);

        }
        m_table.putValue("state", NetworkTableValue.makeString(elevatorPreset));
        m_table.putValue("offset", NetworkTableValue.makeDouble(m_offset));
        m_table.putValue("motor1position",
                NetworkTableValue.makeDouble(m_elevatorMotor1.getRotorPosition().getValueAsDouble()));
        m_table.putValue("motor2position",
                NetworkTableValue.makeDouble(m_elevatorMotor2.getRotorPosition().getValueAsDouble()));
        m_table.putValue("Error",
                NetworkTableValue.makeDouble(m_elevatorMotor1.getClosedLoopError().getValueAsDouble()));
        m_table.putValue("D term",
                NetworkTableValue.makeDouble(m_elevatorMotor1.getClosedLoopDerivativeOutput().getValueAsDouble()));
        m_table.putValue("I term",
                NetworkTableValue.makeDouble(m_elevatorMotor1.getClosedLoopIntegratedOutput().getValueAsDouble()));
        m_table.putValue("P term",
                NetworkTableValue.makeDouble(m_elevatorMotor1.getClosedLoopProportionalOutput().getValueAsDouble()));
        m_table.putValue("Closed loop output",
                NetworkTableValue.makeDouble(m_elevatorMotor1.getClosedLoopOutput().getValueAsDouble()));
        m_table.putValue("Feed forward output",
                NetworkTableValue.makeDouble(m_elevatorMotor1.getClosedLoopFeedForward().getValueAsDouble()));

        m_table.putValue("atBottom", NetworkTableValue.makeBoolean(this.atBottom()));
        m_table.putValue("atIntakePosition", NetworkTableValue.makeBoolean(this.atDownPosition()));

    }
}