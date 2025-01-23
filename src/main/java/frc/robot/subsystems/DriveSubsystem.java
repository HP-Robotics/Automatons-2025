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
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
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

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  public NetworkTable driveTrainTable = inst.getTable("drive-train");
  NetworkTable poseEstimatorTable = inst.getTable("pose-estimator-table");

  StructPublisher<Pose2d> posePublisher = poseEstimatorTable.getStructTopic("Fused Pose", Pose2d.struct).publish();

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
  public boolean m_pathplannerUsingNoteVision = false;
  public boolean m_pathPlannerCancelIfNoteSeen = false;

  public final Field2d m_field = new Field2d();
  public final Field2d m_currentPose = new Field2d();
  public final Field2d m_targetPose = new Field2d();

  private final Pigeon2 m_pGyro = new Pigeon2(IDConstants.PigeonID, "CANivore");

  private StatusSignal<Angle> m_pGyroPitch = m_pGyro.getPitch();
  private StatusSignal<Angle> m_pGyroYaw = m_pGyro.getYaw();
  private StatusSignal<Angle> m_pGyroRoll = m_pGyro.getRoll();

  PIDController rotationController;
  PoseEstimatorSubsystem m_poseEstimator;

  StructPublisher<Pose2d> drivePublisher;
  private double m_gyroOffset;
  RobotConfig config;

  public DriveSubsystem(PoseEstimatorSubsystem poseEstimator) {
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> driveRobotRelative(speeds, feedforwards), // Method that will drive the robot given
                                                                            // ROBOT RELATIVE ChassisSpeeds. Also
                                                                            // optionally outputs individual module
                                                                            // feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic
                                        // drive trains
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        ),
        config, // The robot configuration
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
    rotationController = new PIDController(DriveConstants.turningControllerkP, DriveConstants.turningControllerkI,
        DriveConstants.turningControllerkD);
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.setTolerance(DriveConstants.turningControllerTolerance);
    rotationController.setIZone(DriveConstants.turningControllerIZone);

    drivePublisher = poseEstimatorTable.getStructTopic("Drive Pose", Pose2d.struct).publish();

    if (SubsystemConstants.useDataManager) {
      // SignalLogger.start();
    }

  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(m_swerveModuleStates);
  }

  public void updateOdometry() {
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
    // driveTrainTable.putValue("Robot x",
    // NetworkTableValue.makeDouble(m_poseEstimator.getPose().getX()));
    // driveTrainTable.putValue("Robot y",
    // NetworkTableValue.makeDouble(m_poseEstimator.getPose().getY()));
    // driveTrainTable.putValue("Robot theta",
    // NetworkTableValue.makeDouble(m_poseEstimator.getPose().getRotation().getDegrees()));
    driveTrainTable.putValue("is Drive Abs Working", NetworkTableValue.makeBoolean(
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
    driveTrainTable.putValue("Pigeon Pitch", NetworkTableValue.makeDouble(m_pGyroPitch.getValueAsDouble()));
    driveTrainTable.putValue("Pigeon Yaw", NetworkTableValue.makeDouble(m_pGyroYaw.getValueAsDouble()));
    driveTrainTable.putValue("Pigeon Roll", NetworkTableValue.makeDouble(m_pGyroRoll.getValueAsDouble()));
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

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, pigeonYaw)
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeed);
    setModuleStates(swerveModuleStates);
  }

  public void driveRobotRelative(ChassisSpeeds speeds, DriveFeedforwards feedForwards) {
    ChassisSpeeds discSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(discSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeed);
    setModuleStates(swerveModuleStates);
  }

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
        MathUtil.applyDeadband(joystick.getRawAxis(0), ControllerConstants.driveJoystickDeadband)
            * -1
            * DriveConstants.kMaxAngularSpeed,
        m_fieldRelative);
  }

  public void drivePointedTowardsAngle(CommandJoystick joystick, Rotation2d targetAngle) {
    double rot = rotationController.calculate(m_poseEstimator.getPose().getRotation().getRadians(),
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

    driveTrainTable.putValue("Rotation Current Angle",
        NetworkTableValue.makeDouble(m_poseEstimator.getPose().getRotation().getDegrees()));
    driveTrainTable.putValue("Rotation Target Angle", NetworkTableValue.makeDouble(targetAngle.getDegrees()));
    driveTrainTable.putValue("Rotation Power Input", NetworkTableValue.makeDouble(rot));

    driveTrainTable.putValue("Rotation Controller P",
        NetworkTableValue.makeDouble(rotationController.getP() * rotationController.getError()));
    driveTrainTable.putValue("Rotation Controller Position Error",
        NetworkTableValue.makeDouble(rotationController.getError()));
    driveTrainTable.putValue("Rotation Controller Setpoint",
        NetworkTableValue.makeDouble(rotationController.getSetpoint()));
  }

  public void driveForwardWithAngle(double speed, Rotation2d noteAngle) {
    double rot = rotationController.calculate(m_poseEstimator.getPose().getRotation().getRadians(),
        noteAngle.getRadians());
    drive(
        speed * DriveConstants.kMaxSpeed * Math.max(1 - Math.abs(rot), 0),
        0, // speed * -1 * DriveConstants.kMaxSpeed,
        rot * DriveConstants.kMaxAngularSpeed,
        false);

    driveTrainTable.putValue("Rotation Current Angle",
        NetworkTableValue.makeDouble(m_poseEstimator.getPose().getRotation().getDegrees()));
    driveTrainTable.putValue("Rotation Target Angle", NetworkTableValue.makeDouble(noteAngle.getDegrees()));
    driveTrainTable.putValue("Rotation Power Input", NetworkTableValue.makeDouble(rot));

    driveTrainTable.putValue("Rotation Controller P",
        NetworkTableValue.makeDouble(rotationController.getP() * rotationController.getError()));
    driveTrainTable.putValue("Rotation Controller Position Error",
        NetworkTableValue.makeDouble(rotationController.getError()));
    driveTrainTable.putValue("Rotation Controller Setpoint",
        NetworkTableValue.makeDouble(rotationController.getSetpoint()));
  }

  public boolean pointedTowardsAngle() {
    return rotationController.atSetpoint();
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
    m_backRight.setDesiredState(swerveModuleStates[2]);
    m_backLeft.setDesiredState(swerveModuleStates[3]);
  }

  public void setFieldRelative(boolean isTrue) {
    m_fieldRelative = isTrue;
  }

  public void toggleFieldRelative() {
    m_fieldRelative = !m_fieldRelative;
  }

  public void initializePoseEstimator(Pose2d pose) {
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
    // setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(new
    // ChassisSpeeds()));

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
    // m_pGyro.setYaw(angle);// TODO how do we want this to interact with pose
    // estimator?
    // add offset (and a wraparound to avoid the offset breaking things)
  }

  public double getYaw() {
    return MathUtil.inputModulus(m_pGyro.getYaw().getValueAsDouble() - m_gyroOffset, -180.0, 180.0);
  }

  public void resetYaw() {
    resetYaw(0);
  }
}
