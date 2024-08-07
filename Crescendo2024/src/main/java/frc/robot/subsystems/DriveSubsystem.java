// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;

public class DriveSubsystem extends SubsystemBase {

  private final CANSparkMax m_leftLeaderMotor = new CANSparkMax(DriveConstants.leftBackCAN,
      CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_leftFollowerMotor = new CANSparkMax(DriveConstants.leftFrontCAN,
      CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_rightLeaderMotor = new CANSparkMax(DriveConstants.rightFrontCAN,
      CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_rightFollowerMotor = new CANSparkMax(DriveConstants.rightBackCAN,
      CANSparkMax.MotorType.kBrushless);

  private final SparkPIDController leftSpeedPID = m_leftLeaderMotor.getPIDController();
  private final SparkPIDController rightSpeedPID = m_rightLeaderMotor.getPIDController();

  private final DifferentialDrive m_drive;

  //DECLARATION OF ENCODERS
  private final RelativeEncoder m_leftEncoder = m_leftLeaderMotor.getEncoder();
  private final RelativeEncoder m_rightEncoder = m_rightLeaderMotor.getEncoder();

  private static AHRS ahrs = new AHRS(SerialPort.Port.kUSB);

  private final AnalogGyro m_gyro = new AnalogGyro(0);

  private DifferentialDriveOdometry m_odometry;

  private boolean isScanningField = false;
  private double startingX = 0;
  private double startingY = 0;
  private double startingAngle = 0;

  private double angleFromTag1;
  private double angleFromTag2;
  private double distanceFromTag;
  private double distanceBetweenTagsRed = 22.1875/39.37; //DIST BETWEEN 3 AND 4
  private double distanceBetweenTagsBlue = 22.1875/39.37; //DIST BETWEEN 7 AND 8
  private boolean autoAimIsReady = false;

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

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

   private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              (Measure<Voltage> volts) -> {
                m_leftLeaderMotor.setVoltage(volts.in(Volts));
                m_rightLeaderMotor.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_leftLeaderMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(m_leftEncoder.getPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(m_leftEncoder.getVelocity(), MetersPerSecond));
                // Record a frame for the right motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_rightLeaderMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(m_rightEncoder.getPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(m_rightEncoder.getVelocity(), MetersPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));

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


    //SET ENCODER CONVERSION
    m_leftEncoder.setPositionConversionFactor(
        (DriveConstants.wheelDiameterMetres * Math.PI / (DriveConstants.gearRatio)));
    m_rightEncoder.setPositionConversionFactor(
        (DriveConstants.wheelDiameterMetres * Math.PI / (DriveConstants.gearRatio)));

    m_leftEncoder.setVelocityConversionFactor((DriveConstants.wheelDiameterMetres * Math.PI / (DriveConstants.gearRatio * 60)));
    m_rightEncoder.setVelocityConversionFactor((DriveConstants.wheelDiameterMetres * Math.PI / (DriveConstants.gearRatio * 60)));

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
          m_leftEncoder.getPosition(),
          m_rightEncoder.getPosition(),
          new Pose2d(startingX, startingY, Rotation2d.fromDegrees(startingAngle)));
    }

    m_driveTrainSim = DifferentialDrivetrainSim.createKitbotSim( // CHANGE AS NEEDED!!
        KitbotMotor.kDualCIMPerSide,
        KitbotGearing.k10p71,
        KitbotWheelSize.kSixInch,
        null);

    m_fieldSim = new Field2d();
    // SmartDashboard.putData("Field", m_fieldSim);

    m_leftEncoderSim = new EncoderSim(m_leftEncoderObj);
    m_rightEncoderSim = new EncoderSim(m_rightEncoderObj);
    m_gyroSim = new AnalogGyroSim(m_gyro);


    //PATHPLANNER DECLARATION
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

    // SmartDashboard.putNumber("Current Left leader", m_leftLeaderMotor.getOutputCurrent());
    // SmartDashboard.putNumber("Current Left follower", m_leftFollowerMotor.getOutputCurrent());
    // SmartDashboard.putNumber("Current Right leader", m_rightLeaderMotor.getOutputCurrent());
    // SmartDashboard.putNumber("Current Right follower", m_rightFollowerMotor.getOutputCurrent());

    SmartDashboard.putBoolean("auto aim", autoAimIsReady);

    if(DriverStation.isDisabled()){
      resetEncoders();
      ahrs.reset();
    }

    if (Robot.isSimulation()) {
      m_odometry.update(
          m_gyro.getRotation2d(),
          m_leftEncoderObj.getDistance(),
          m_rightEncoderObj.getDistance());
    } else {
      if (m_camera.getLatestResult().hasTargets() && isScanningField) {
        updatePoseFromCamera(m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
        resetEncoders();
        ahrs.reset();
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
            m_leftEncoder.getPosition(),
            m_rightEncoder.getPosition());
      }

    }
    
    m_fieldSim.setRobotPose(getPose());

    if(m_camera.getLatestResult().hasTargets()){
      List <PhotonTrackedTarget> targets = m_camera.getLatestResult().getTargets();
        // SmartDashboard.putNumber("ID1", targets.get(0).getFiducialId());
        // SmartDashboard.putNumber("ID2", targets.get(1).getFiducialId());

      if(targets.size() >= 2 && (targets.get(0).getFiducialId() == 4 || targets.get(1).getFiducialId() == 4)){
        if(targets.get(0).getFiducialId() == 4){
          angleFromTag1 = -1 * targets.get(0).getYaw();
          angleFromTag2 = targets.get(1).getYaw();
        }else{
          angleFromTag1 = -1 * targets.get(1).getYaw();
          angleFromTag2 = targets.get(0).getYaw();
        }

        distanceFromTag = distanceBetweenTagsRed/(Math.sin(angleFromTag1 * Math.PI / 180) + Math.cos(angleFromTag1 * Math.PI / 180) * Math.tan(angleFromTag2 * Math.PI / 180));

        if(distanceFromTag < 3.6){
          autoAimIsReady = true;
        }else{
          autoAimIsReady = false;
        }

      }else if(targets.size() >= 2 && (targets.get(0).getFiducialId() == 7 || targets.get(1).getFiducialId() == 7)){
        if(targets.get(0).getFiducialId() == 7){
          angleFromTag1 = -1 * targets.get(0).getYaw();
          angleFromTag2 = targets.get(1).getYaw();
        }else{
          angleFromTag1 = -1 * targets.get(1).getYaw();
          angleFromTag2 = targets.get(0).getYaw();
        }

        distanceFromTag = -distanceBetweenTagsBlue/(Math.sin(angleFromTag1 * Math.PI / 180) + Math.cos(angleFromTag1 * Math.PI / 180) * Math.tan(angleFromTag2 * Math.PI / 180));

        if(distanceFromTag < 3.6){
          autoAimIsReady = true;
        }else{
          autoAimIsReady = false;
        }

      }else{
        autoAimIsReady = false;
      }

    }else{
      autoAimIsReady = false;
    }

    // SmartDashboard.putBoolean("hasAprilTag", m_camera.getLatestResult().hasTargets());
    // SmartDashboard.putBoolean("Is Scanning", isScanningField);
    // SmartDashboard.putNumber("middle tag angle", angleFromTag1);
    // SmartDashboard.putNumber("side tag angle", angleFromTag2);
    SmartDashboard.putNumber("dist from tag", distanceFromTag);

    
    SmartDashboard.putNumber("Yaw", ahrs.getAngle());
    // SmartDashboard.putNumber("Right encoder", m_rightEncoder.getPosition());
    // SmartDashboard.putNumber("Left encoder", m_leftEncoder.getPosition());
    // SmartDashboard.putNumber("Estimated X", m_fieldSim.getRobotPose().getX());
    // SmartDashboard.putNumber("Estimated Y", m_fieldSim.getRobotPose().getY());
    // SmartDashboard.putNumber("Estimated Rotation", m_fieldSim.getRobotPose().getRotation().getDegrees());
    // SmartDashboard.putNumber("Right velocity", m_rightEncoder.getVelocity());
    // SmartDashboard.putNumber("Left velocity", m_leftEncoder.getVelocity());


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


    m_drive.arcadeDrive(sign * Math.pow(xForward, 2), 0.8 * Math.pow(zRotation, 3)
    );
  }

  public void driveRaw(double power) {
    m_leftLeaderMotor.set(power);
    m_rightLeaderMotor.set(power);
  }

  public void driveSpeed(double speed){
    leftSpeedPID.setReference(speed, ControlType.kVelocity);
    rightSpeedPID.setReference(speed, ControlType.kVelocity);

  }

  public double getDistanceFromSpeaker(){
    return distanceFromTag;
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


  public double getEstimatedRotation(){
    return m_poseEstimator.getEstimatedPosition().getRotation().getDegrees();
  }

  public double modAngle(double angle) {
    return Math.IEEEremainder(angle, 360);
  }

  public void setFieldScan(boolean isScanning){
    isScanningField = isScanning;
  }

  public void setStartingPosition(Double degrees, double x, double y){
    m_odometry.resetPosition(
      ahrs.getRotation2d(),
      m_leftEncoder.getPosition(),
      m_rightEncoder.getPosition(),
      new Pose2d(x, y, Rotation2d.fromDegrees(degrees))
    );
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction){
    return m_sysIdRoutine.dynamic(direction);
  }

}
