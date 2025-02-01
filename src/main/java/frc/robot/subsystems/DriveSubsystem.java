/// Copyright (c) FIRST and other WPILib contributors.
//\ Open Source Software; you can modify and/or share it under the terms of
//     the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.*;
import frc.robot.SwerveModule;

/** Represents a swerve drive style drivetrain. */
public class DriveSubsystem extends SubsystemBase {
  SwerveDriveOdometry m_odometry;

  PPHolonomicDriveController m_driveController;

  SwerveSetpointGenerator m_setpointGenerator;
  SwerveSetpoint m_previousSetpoint;

  NetworkTableInstance m_inst = NetworkTableInstance.getDefault();
  public NetworkTable m_driveTrainTable = m_inst.getTable("drive-train");
  NetworkTable m_poseEstimatorTable = m_inst.getTable("pose-estimator-table");

  StructPublisher<Pose2d> m_posePublisher = m_poseEstimatorTable.getStructTopic("Fused Pose", Pose2d.struct).publish();

  // BIG BONGO 7
  private final SwerveModule m_frontLeft = new SwerveModule(IDConstants.FLDriveMotorID,
      IDConstants.FLTurningMotorID, PortConstants.FLAbsEncoder, DriveConstants.absEncoderForwardFL, "FL");
  // BIG BONGO 2
  private final SwerveModule m_frontRight = new SwerveModule(IDConstants.FRDriveMotorID,
      IDConstants.FRTurningMotorID, PortConstants.FRAbsEncoder, DriveConstants.absEncoderForwardFR, "FR");
  // BIG BONGO 1
  private final SwerveModule m_backRight = new SwerveModule(IDConstants.BRDriveMotorID,
      IDConstants.BRTurningMotorID, PortConstants.BRAbsEncoder, DriveConstants.absEncoderForwardBR, "BR");
  // BIG BONGO 3
  private final SwerveModule m_backLeft = new SwerveModule(IDConstants.BLDriveMotorID,
      IDConstants.BLTurningMotorID, PortConstants.BLAbsEncoder, DriveConstants.absEncoderForwardBL, "BL");

  private SwerveModuleState[] m_swerveModuleStates = {
      new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()
  };

  public boolean m_fieldRelative = true;

  public final Field2d m_field = new Field2d();
  public final Field2d m_currentPose = new Field2d();
  public final Field2d m_targetPose = new Field2d();

  private final Pigeon2 m_pGyro = new Pigeon2(IDConstants.PigeonID, "CANivore");

  private StatusSignal<Angle> m_pGyroPitch = m_pGyro.getPitch();
  private StatusSignal<Angle> m_pGyroYaw = m_pGyro.getYaw();
  private StatusSignal<Angle> m_pGyroRoll = m_pGyro.getRoll();

  PIDController m_rotationController;
  PIDController m_xController;
  PIDController m_yController;
  PoseEstimatorSubsystem m_poseEstimator;

  StructPublisher<Pose2d> m_drivePublisher;
  private double m_gyroOffset;
  RobotConfig m_config;

  public DriveSubsystem(PoseEstimatorSubsystem poseEstimator) {
    m_pGyro.getYaw().setUpdateFrequency(DriveConstants.odometryUpdateFrequency);

    try {
      m_config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> driveRobotRelative(speeds, feedforwards),
        // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        // Also optionally outputs individual module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic
                                        // drive trains
            PathplannerPIDConstants.translationConstants, // Translation PID constants
            PathplannerPIDConstants.rotationConstants // Rotation PID constants
        ),
        m_config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );
    m_poseEstimator = poseEstimator;
    m_pGyro.setYaw(0);

    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      m_currentPose.setRobotPose(pose);
    });
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      m_targetPose.setRobotPose(pose);
    });

    SmartDashboard.putData("Field", m_field);
    m_rotationController = new PIDController(DriveConstants.rotationControllerkP, DriveConstants.rotationControllerkI,
        DriveConstants.rotationControllerkD);
    m_rotationController.enableContinuousInput(-Math.PI, Math.PI);
    m_rotationController.setTolerance(DriveConstants.rotationControllerTolerance);
    m_rotationController.setIZone(DriveConstants.rotationControllerIZone);

    m_xController = new PIDController(DriveConstants.XControllerkP, DriveConstants.XControllerkI,
        DriveConstants.XControllerkD);
    m_xController.setTolerance(DriveConstants.XControllerTolerance);
    m_xController.setIZone(DriveConstants.XControllerIZone);
    m_yController = new PIDController(DriveConstants.YControllerkP, DriveConstants.YControllerkI,
        DriveConstants.YControllerkD);
    m_yController.setTolerance(DriveConstants.YControllerTolerance);
    m_yController.setIZone(DriveConstants.YControllerIZone);

    m_drivePublisher = m_poseEstimatorTable.getStructTopic("Drive Pose", Pose2d.struct).publish();

    if (SubsystemConstants.useDataManager) {
      // SignalLogger.start(); //Phoenix hoot logger
    }

    m_setpointGenerator = new SwerveSetpointGenerator(
        m_config, // The robot configuration
        // This is the same config used for generating trajectories and running path
        // following commands
        Units.rotationsToRadians(DriveConstants.kModuleMaxAngularSpeed)
    // The max rotation velocity of a swerve module in radians per second
    );

    m_previousSetpoint = new SwerveSetpoint(getRobotRelativeSpeeds(), m_swerveModuleStates,
        DriveFeedforwards.zeros(m_config.numModules));

  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(m_swerveModuleStates);
  }

  public void updateOdometry() {
    StatusSignal.waitForAll(1.0 / DriveConstants.odometryUpdateFrequency,
        m_frontLeft.m_driveMotor.getRotorPosition(),
        m_frontRight.m_driveMotor.getRotorPosition(),
        m_backLeft.m_driveMotor.getRotorPosition(),
        m_backRight.m_driveMotor.getRotorPosition(),
        m_pGyro.getYaw());
    var pigeonYaw = new Rotation2d(Math.toRadians(getYaw()));
    // Update the odometry in the periodic block
    if (m_poseEstimator != null) {
      m_poseEstimator.updatePoseEstimator(pigeonYaw, new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backRight.getPosition(),
          m_backLeft.getPosition()
      });
    }
  }

  @Override
  public void periodic() {
    if (getPose() != null) {
      m_field.setRobotPose(getPose());
    }

    m_driveTrainTable.putValue("Robot theta",
        NetworkTableValue.makeDouble(m_poseEstimator.getPose().getRotation().getDegrees()));
    m_driveTrainTable.putValue("is Drive Abs Working", NetworkTableValue.makeBoolean(
        m_backLeft.m_absEncoder.get() != 0 &&
            m_backRight.m_absEncoder.get() != 0 &&
            m_frontLeft.m_absEncoder.get() != 0 &&
            m_frontRight.m_absEncoder.get() != 0));

    // TODO investigate why this takes so long
    m_frontLeft.updateShuffleboard();
    m_frontRight.updateShuffleboard();
    m_backRight.updateShuffleboard();
    m_backLeft.updateShuffleboard();

    // BaseStatusSignal.refreshAll(m_pGyroPitch, m_pGyroYaw, m_pGyroRoll);
    m_driveTrainTable.putValue("Pigeon Pitch", NetworkTableValue.makeDouble(m_pGyroPitch.getValueAsDouble()));
    m_driveTrainTable.putValue("Pigeon Yaw", NetworkTableValue.makeDouble(m_pGyroYaw.getValueAsDouble()));
    m_driveTrainTable.putValue("Pigeon Roll", NetworkTableValue.makeDouble(m_pGyroRoll.getValueAsDouble()));
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the ro bot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var pigeonYaw = new Rotation2d(Math.toRadians(getYaw()));

    m_previousSetpoint = m_setpointGenerator.generateSetpoint(
        m_previousSetpoint, // The previous setpoint
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, pigeonYaw)
            : new ChassisSpeeds(xSpeed, ySpeed, rot), // The desired target speeds
        TimedRobot.kDefaultPeriod // The loop time of the robot code, in seconds
    );
    setModuleStates(m_previousSetpoint.moduleStates()); // Method that will drive the robot given target module states
  }

  public void driveRobotRelative(ChassisSpeeds speeds, DriveFeedforwards feedForwards) {

    m_previousSetpoint = m_setpointGenerator.generateSetpoint(
        m_previousSetpoint, // The previous setpoint
        speeds, // The desired target speeds
        TimedRobot.kDefaultPeriod // The loop time of the robot code, in seconds
    );
    setModuleStates(m_previousSetpoint.moduleStates()); // Method that will drive the robot given target module states
  }

  // TODO: Refactor + get rid of magic numbers
  public void driveWithJoystick(CommandJoystick joystick) {
    drive(
        Math.signum(joystick.getRawAxis(1))

            * Math.pow(MathUtil.applyDeadband(joystick.getRawAxis(1),
                ControllerConstants.driveJoystickDeadband), ControllerConstants.driveJoystickExponent)
            * -1 * DriveConstants.kMaxSpeed,
        Math.signum(joystick.getRawAxis(0))
            * Math.pow(MathUtil.applyDeadband(joystick.getRawAxis(0),
                ControllerConstants.driveJoystickDeadband), ControllerConstants.driveJoystickExponent)
            * -1 * DriveConstants.kMaxSpeed,
        MathUtil.applyDeadband(joystick.getRawAxis(4), ControllerConstants.driveJoystickDeadband)
            * -1
            * DriveConstants.kMaxAngularSpeed,
        m_fieldRelative);
  }

  public void drivePointedTowardsAngle(CommandJoystick joystick, Rotation2d targetAngle) {
    double rot = m_rotationController.calculate(m_poseEstimator.getPose().getRotation().getRadians(),
        targetAngle.getRadians());
    drive(
        Math.signum(joystick.getRawAxis(1))
            * Math.pow(MathUtil.applyDeadband(joystick.getRawAxis(1),
                ControllerConstants.driveJoystickDeadband), 2)
            * -1 * DriveConstants.kMaxSpeed,
        Math.signum(joystick.getRawAxis(0))
            * Math.pow(MathUtil.applyDeadband(joystick.getRawAxis(0),
                ControllerConstants.driveJoystickDeadband), 2)
            * -1 * DriveConstants.kMaxSpeed,
        rot * 1 * DriveConstants.kMaxAngularSpeed,
        m_fieldRelative);

    m_driveTrainTable.putValue("Rotation Current Angle",
        NetworkTableValue.makeDouble(m_poseEstimator.getPose().getRotation().getDegrees()));
    m_driveTrainTable.putValue("Rotation Target Angle", NetworkTableValue.makeDouble(targetAngle.getDegrees()));
    m_driveTrainTable.putValue("Rotation Power Input", NetworkTableValue.makeDouble(rot));

    m_driveTrainTable.putValue("Rotation Controller P",
        NetworkTableValue.makeDouble(m_rotationController.getP() * m_rotationController.getError()));
    m_driveTrainTable.putValue("Rotation Controller Position Error",
        NetworkTableValue.makeDouble(m_rotationController.getError()));
    m_driveTrainTable.putValue("Rotation Controller Setpoint",
        NetworkTableValue.makeDouble(m_rotationController.getSetpoint()));
  }

  public void driveToPose(Pose2d target) {
    double rot = m_rotationController.calculate(m_poseEstimator.getPose().getRotation().getRadians(),
        target.getRotation().getRadians());
    double x = -MathUtil.clamp(m_xController.calculate(m_poseEstimator.getPose().getX(),
        target.getX()), -1, 1);
    double y = -MathUtil.clamp(m_yController.calculate(m_poseEstimator.getPose().getY(),
        target.getY()), -1, 1);
    System.out.println(x + ", " + y);
    drive(
        MathUtil.isNear(0, x, 0.07) ? 0 : x * -1 * DriveConstants.kMaxSpeed,
        MathUtil.isNear(0, y, 0.07) ? 0 : y * -1 * DriveConstants.kMaxSpeed,
        rot * DriveConstants.kMaxAngularSpeed,
        true);
    m_driveTrainTable.putValue("X Error", NetworkTableValue.makeDouble(m_xController.getError()));
    m_driveTrainTable.putValue("X P Contribution",
        NetworkTableValue.makeDouble(m_xController.getError() * m_xController.getP()));
    m_driveTrainTable.putValue("X I Contribution",
        NetworkTableValue.makeDouble(m_xController.getAccumulatedError() * m_xController.getI()));
    m_driveTrainTable.putValue("X D Contribution",
        NetworkTableValue.makeDouble(m_xController.getErrorDerivative() * m_xController.getD()));

    m_driveTrainTable.putValue("Theta Error", NetworkTableValue.makeDouble(m_rotationController.getError()));
    m_driveTrainTable.putValue("Theta P Contribution",
        NetworkTableValue.makeDouble(m_rotationController.getError() * m_rotationController.getP()));
    m_driveTrainTable.putValue("Theta I Contribution",
        NetworkTableValue.makeDouble(m_rotationController.getAccumulatedError() * m_rotationController.getI()));
    m_driveTrainTable.putValue("Theta D Contribution",
        NetworkTableValue.makeDouble(m_rotationController.getErrorDerivative() * m_rotationController.getD()));

    m_driveTrainTable.putValue("Y Error", NetworkTableValue.makeDouble(m_yController.getError()));
    m_driveTrainTable.putValue("Y P Contribution",
        NetworkTableValue.makeDouble(m_yController.getError() * m_yController.getP()));
    m_driveTrainTable.putValue("Y I Contribution",
        NetworkTableValue.makeDouble(m_yController.getAccumulatedError() * m_yController.getI()));
    m_driveTrainTable.putValue("Y D Contribution",
        NetworkTableValue.makeDouble(m_yController.getErrorDerivative() * m_yController.getD()));
  }

  public void driveForwardWithAngle(double speed, Rotation2d noteAngle) {
    double rot = m_rotationController.calculate(m_poseEstimator.getPose().getRotation().getRadians(),
        noteAngle.getRadians());
    drive(
        speed * DriveConstants.kMaxSpeed * Math.max(1 - Math.abs(rot), 0),
        0, // speed * -1 * DriveConstants.kMaxSpeed,
        rot * DriveConstants.kMaxAngularSpeed,
        false);

    m_driveTrainTable.putValue("Rotation Current Angle",
        NetworkTableValue.makeDouble(m_poseEstimator.getPose().getRotation().getDegrees()));
    m_driveTrainTable.putValue("Rotation Target Angle", NetworkTableValue.makeDouble(noteAngle.getDegrees()));
    m_driveTrainTable.putValue("Rotation Power Input", NetworkTableValue.makeDouble(rot));

    m_driveTrainTable.putValue("Rotation Controller P",
        NetworkTableValue.makeDouble(m_rotationController.getP() * m_rotationController.getError()));
    m_driveTrainTable.putValue("Rotation Controller Position Error",
        NetworkTableValue.makeDouble(m_rotationController.getError()));
    m_driveTrainTable.putValue("Rotation Controller Setpoint",
        NetworkTableValue.makeDouble(m_rotationController.getSetpoint()));
  }

  public boolean pointedTowardsAngle() {
    return m_rotationController.atSetpoint();
  }

  public Pose2d getPose() {
    return m_poseEstimator.getPose();
  }

  public ChassisSpeeds getCurrentspeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(m_swerveModuleStates);
  }

  public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    this.m_swerveModuleStates = swerveModuleStates;
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void setFieldRelative(boolean isTrue) {
    m_fieldRelative = isTrue;
  }

  public void toggleFieldRelative() {
    m_fieldRelative = !m_fieldRelative;
  }

  public void startPoseEstimator(Pose2d pose) {
    if (m_poseEstimator != null) {
      m_poseEstimator.createPoseEstimator(DriveConstants.kDriveKinematics,
          new Rotation2d(Math.toRadians(getYaw())), new SwerveModulePosition[] {
              m_frontLeft.getPosition(),
              m_frontRight.getPosition(),
              m_backRight.getPosition(),
              m_backLeft.getPosition()
          }, pose);
    }
  }

  public void resetOffsets() { // Turn encoder offset
    m_frontLeft.resetOffset();
    m_frontRight.resetOffset();
    m_backRight.resetOffset();
    m_backLeft.resetOffset();
  }

  public void resetPose(Pose2d pose) {
    if (pose == null) {
      return;
    }
    var pigeonYaw = new Rotation2d(Math.toRadians(getYaw()));
    m_poseEstimator.resetPosition(
        pigeonYaw,
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backRight.getPosition(),
            m_backLeft.getPosition()
        },
        pose);
  }

  public void resetPoseEstimatorHeading(Rotation2d angle) {
    if (angle == null) {
      return;
    }
    Pose2d pose = new Pose2d(m_poseEstimator.getPose().getTranslation(), angle);
    var pigeonYaw = new Rotation2d(Math.toRadians(getYaw()));
    m_poseEstimator.resetPosition(
        pigeonYaw,
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backRight.getPosition(),
            m_backLeft.getPosition()
        },
        pose);
  }

  /**
   * Resets robot's conception of field orientation
   */
  public void resetYaw(double angle) {
    m_gyroOffset = m_pGyro.getYaw().getValueAsDouble() - angle;
  }

  public double getYaw() {
    return MathUtil.inputModulus(m_pGyro.getYaw().getValueAsDouble() - m_gyroOffset, -180.0, 180.0);
  }

  public void resetYaw() {
    resetYaw(0);
  }
}
