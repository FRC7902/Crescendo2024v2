// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import org.photonvision.PhotonCamera;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;

public class DriveSubsystem extends SubsystemBase {

  private final CANSparkMax m_leftLeaderMotor = new CANSparkMax(DriveConstants.leftFrontCAN,
      CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_leftFollowerMotor = new CANSparkMax(DriveConstants.leftBackCAN,
      CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_rightLeaderMotor = new CANSparkMax(DriveConstants.rightFrontCAN,
      CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_rightFollowerMotor = new CANSparkMax(DriveConstants.rightBackCAN,
      CANSparkMax.MotorType.kBrushless);

  private final DifferentialDrive m_drive;

  private final RelativeEncoder m_leftEncoder = m_leftLeaderMotor.getEncoder();
  private final RelativeEncoder m_rightEncoder = m_rightLeaderMotor.getEncoder();

  private static AHRS ahrs = new AHRS(SerialPort.Port.kUSB);

  private final AnalogGyro m_gyro = new AnalogGyro(0);

  private DifferentialDriveOdometry m_odometry;

  private boolean isScanningField = false;

  // Simulation Stuff
  private final Encoder m_leftEncoderObj = new Encoder(0, 1);
  private final Encoder m_rightEncoderObj = new Encoder(2, 3);

  private EncoderSim m_leftEncoderSim;
  private EncoderSim m_rightEncoderSim;
  private AnalogGyroSim m_gyroSim;
  public DifferentialDrivetrainSim m_driveTrainSim;
  private Field2d m_fieldSim;
  private PhotonCamera m_camera;

  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(
      DriveConstants.trackWidthInches);
  private final DifferentialDrivePoseEstimator m_poseEstimator = new DifferentialDrivePoseEstimator(
      m_kinematics,
      ahrs.getRotation2d(),
      m_leftEncoder.getPosition(),
      m_rightEncoder.getPosition(),
      new Pose2d(),
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)), // state std devs
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))// cam std devs
  );

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(PhotonCamera camera) {
    m_camera = camera;

    m_leftLeaderMotor.restoreFactoryDefaults();
    m_leftFollowerMotor.restoreFactoryDefaults();
    m_rightLeaderMotor.restoreFactoryDefaults();
    m_rightFollowerMotor.restoreFactoryDefaults();

    m_rightLeaderMotor.setInverted(false);
    m_leftLeaderMotor.setInverted(true);

    m_leftFollowerMotor.follow(m_leftLeaderMotor);
    m_rightFollowerMotor.follow(m_rightLeaderMotor);

    m_drive = new DifferentialDrive(m_leftLeaderMotor, m_rightLeaderMotor);

    m_leftEncoder.setPositionConversionFactor(
        -DriveConstants.wheelDiameterMetres * Math.PI / DriveConstants.gearRatio);
    m_rightEncoder.setPositionConversionFactor(
        DriveConstants.wheelDiameterMetres * Math.PI / DriveConstants.gearRatio);


    m_leftEncoderObj.setDistancePerPulse(0.1524 * Math.PI / 1024);
    m_rightEncoderObj.setDistancePerPulse(0.1524 * Math.PI / 1024);

    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
    ahrs.reset();

    m_rightLeaderMotor.setIdleMode(IdleMode.kBrake);
    m_rightFollowerMotor.setIdleMode(IdleMode.kBrake);
    m_leftLeaderMotor.setIdleMode(IdleMode.kBrake);
    m_leftFollowerMotor.setIdleMode(IdleMode.kBrake);


    m_leftLeaderMotor.setSmartCurrentLimit(45);
    m_leftFollowerMotor.setSmartCurrentLimit(45);
    m_rightLeaderMotor.setSmartCurrentLimit(45);
    m_rightFollowerMotor.setSmartCurrentLimit(45);

    if (Robot.isSimulation()) {
      m_odometry = new DifferentialDriveOdometry(
          m_gyro.getRotation2d(),
          m_leftEncoderObj.getDistance(),
          m_rightEncoderObj.getDistance(),
          new Pose2d(1, 1, new Rotation2d()));
    } else {
      m_odometry = new DifferentialDriveOdometry(
          m_gyro.getRotation2d(),
          -m_leftEncoder.getPosition(),
          -m_rightEncoder.getPosition(),
          new Pose2d(1, 1, new Rotation2d()));
    }

    m_driveTrainSim = DifferentialDrivetrainSim.createKitbotSim( // CHANGE AS NEEDED!!
        KitbotMotor.kDualCIMPerSide,
        KitbotGearing.k10p71,
        KitbotWheelSize.kSixInch,
        null);

    m_fieldSim = new Field2d();
    SmartDashboard.putData("Field", m_fieldSim);

    m_leftEncoderSim = new EncoderSim(m_leftEncoderObj);
    m_rightEncoderSim = new EncoderSim(m_rightEncoderObj);
    m_gyroSim = new AnalogGyroSim(m_gyro);

    AutoBuilder.configureRamsete(
        this::getPose,
        this::resetPose,
        this::getWheelSpeeds,
        this::driveSpeeds, new ReplanningConfig(), // Default path replanning config. See the API for the options here
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
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Current Left leader", m_leftLeaderMotor.getOutputCurrent());
    SmartDashboard.putNumber("Current Left follower", m_leftFollowerMotor.getOutputCurrent());
    SmartDashboard.putNumber("Current Right leader", m_rightLeaderMotor.getOutputCurrent());
    SmartDashboard.putNumber("Current Right follower", m_rightFollowerMotor.getOutputCurrent());

    if (Robot.isSimulation()) {
      m_odometry.update(
          m_gyro.getRotation2d(),
          m_leftEncoderObj.getDistance(),
          m_rightEncoderObj.getDistance());
    } else {
      if (m_camera.getLatestResult().hasTargets() && isScanningField) {
        updatePoseFromCamera(m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
        m_odometry.resetPosition(
            ahrs.getRotation2d(),
            m_leftEncoder.getPosition(),
            m_rightEncoder.getPosition(),
            new Pose2d(
                m_poseEstimator.getEstimatedPosition().getX(),
                m_poseEstimator.getEstimatedPosition().getY(),
                m_poseEstimator.getEstimatedPosition().getRotation()));
      } else {
        m_odometry.update(
            ahrs.getRotation2d(),
            -m_leftEncoder.getPosition(),
            -m_rightEncoder.getPosition());
      }

    }
    
    m_fieldSim.setRobotPose(getPose());

    
    SmartDashboard.putBoolean("hasAprilTag", m_camera.getLatestResult().hasTargets());
    SmartDashboard.putBoolean("Is Scanning", isScanningField);
    
    SmartDashboard.putNumber("Yaw", ahrs.getAngle());
    SmartDashboard.putNumber("Right encoder", m_rightEncoder.getPosition());
    SmartDashboard.putNumber("Left encoder", m_leftEncoder.getPosition());
    SmartDashboard.putNumber("Estimated X", m_fieldSim.getRobotPose().getX());
    SmartDashboard.putNumber("Estimated Y", m_fieldSim.getRobotPose().getY());
    SmartDashboard.putNumber("Estimated Rotation", m_fieldSim.getRobotPose().getRotation().getDegrees());


    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {

    // This method will be called once per scheduler run during simulation
    // connect the motors to update the drivetrain

    m_driveTrainSim.setInputs(
        m_leftLeaderMotor.get() * RobotController.getBatteryVoltage(),
        m_rightLeaderMotor.get() * RobotController.getBatteryVoltage());

    m_driveTrainSim.update(0.02);

    m_leftEncoderSim.setDistance(m_driveTrainSim.getLeftPositionMeters());
    m_rightEncoderSim.setDistance(m_driveTrainSim.getRightPositionMeters());

    m_leftEncoderSim.setRate(m_driveTrainSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setRate(m_driveTrainSim.getRightVelocityMetersPerSecond());

    m_gyroSim.setAngle(-m_driveTrainSim.getHeading().getDegrees());

  }

  public void updatePoseFromCamera(double leftDist, double rightDist) {
    m_poseEstimator.update(ahrs.getRotation2d(), leftDist, rightDist);

    var res = m_camera.getLatestResult();

    if (res.hasTargets()) {
      var imageCaptureTime = res.getTimestampSeconds();
      var camToTargetTrans = res.getBestTarget().getBestCameraToTarget();
      var camPose = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField()
          .getTagPose(res.getBestTarget().getFiducialId()).get().transformBy(camToTargetTrans.inverse());

      m_poseEstimator.addVisionMeasurement(camPose.toPose2d(), imageCaptureTime);
    }
  }

  public void driveArcade(double xForward, double zRotation) {
    int sign;
    if(xForward > 0){
      sign = 1;
    }else{
      sign = -1;
    }


    m_drive.arcadeDrive(sign * Math.pow(xForward, 2), Math.pow(zRotation, 3)
    );
  }

  public void driveRaw(double power) {
    m_leftLeaderMotor.set(power);
    m_rightLeaderMotor.set(power);
  }

  public void driveSpeeds(ChassisSpeeds speeds) {
    this.driveArcade(speeds.vxMetersPerSecond / 4.379976, speeds.omegaRadiansPerSecond / 4.379976);
  }

  public ChassisSpeeds getWheelSpeeds() {
    if (Robot.isSimulation()) {
      return new ChassisSpeeds(m_leftEncoderObj.getRate(), m_rightEncoderObj.getRate(),
          m_gyro.getRate());
    } else {
      return new ChassisSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity(),
          Units.degreesToRadians(m_gyro.getRate()));
    }
  }

  public void turn(double amount) {
    m_leftLeaderMotor.set(amount);
    m_rightLeaderMotor.set(-amount);
  }

  public double getPosition() {
    if (Robot.isSimulation()) {
      return m_rightEncoderObj.getDistance();
    } else {
      return m_rightEncoder.getPosition();
    }
  }

  public void resetEncoders() {
    m_rightEncoder.setPosition(0);
    m_leftEncoder.setPosition(0);
  }

  public void resetPose(Pose2d pose) {
    m_rightEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
    m_odometry.resetPosition(
        m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(), pose);
  }

  public void stopMotors() {
    m_leftLeaderMotor.stopMotor();
    m_rightLeaderMotor.stopMotor();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public double getDisplacementX() {
    return m_odometry.getPoseMeters().getX();
  }

  public double getDisplacementY() {
    return m_odometry.getPoseMeters().getY();
  }

  public double getHeading() {// -180 to 180
    if (Robot.isSimulation()) {
      return Math.IEEEremainder(m_gyro.getAngle(), 360);
    } else {
      return Math.IEEEremainder(ahrs.getAngle(), 360);
    }
  }

  public double modAngle(double angle) {
    return Math.IEEEremainder(angle, 360);
  }

  public void setFieldScan(boolean isScanning){
    isScanningField = isScanning;
  }
}