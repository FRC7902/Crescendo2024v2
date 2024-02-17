// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
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

  private final CANSparkMax m_leftLeaderMotor = new CANSparkMax(DriveConstants.leftFrontCANID,
      CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_leftFollowerMotor = new CANSparkMax(DriveConstants.leftBackCANID,
      CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_rightLeaderMotor = new CANSparkMax(DriveConstants.rightFrontCANID,
      CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_rightFollowerMotor = new CANSparkMax(DriveConstants.rightBackCANID,
      CANSparkMax.MotorType.kBrushless);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftLeaderMotor, m_rightLeaderMotor);

  private final RelativeEncoder m_leftEncoder = m_leftLeaderMotor.getEncoder();
  private final RelativeEncoder m_rightEncoder = m_rightLeaderMotor.getEncoder();

  private static AHRS ahrs = new AHRS(SerialPort.Port.kUSB);

  private final AnalogGyro m_gyro = new AnalogGyro(0);

  private DifferentialDriveOdometry m_odometry;
  private DifferentialDriveKinematics m_kinematics;
  private ChassisSpeeds m_chassisSpeeds;

  // Simulation Stuff
  private final Encoder m_leftEncoderObj = new Encoder(0, 1);
  private final Encoder m_rightEncoderObj = new Encoder(2, 3);

  private EncoderSim m_leftEncoderSim;
  private EncoderSim m_rightEncoderSim;
  private AnalogGyroSim m_gyroSim;
  public DifferentialDrivetrainSim m_driveTrainSim;
  private Field2d m_fieldSim;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    // m_leftLeaderMotor.restoreFactoryDefaults(); //Safety precautions, but
    // reccomended
    // m_leftFollowerMotor.restoreFactoryDefaults();
    // m_rightLeaderMotor.restoreFactoryDefaults();
    // m_rightFollowerMotor.restoreFactoryDefaults();

    m_leftFollowerMotor.follow(m_leftLeaderMotor);
    m_rightFollowerMotor.follow(m_rightLeaderMotor);

    m_leftEncoder.setPositionConversionFactor(
        DriveConstants.wheelDiameterMetres * Math.PI / DriveConstants.gearRatio);
    m_rightEncoder.setPositionConversionFactor(
        DriveConstants.wheelDiameterMetres * Math.PI / DriveConstants.gearRatio);

    m_leftEncoderObj.setDistancePerPulse(0.1524 * Math.PI / 1024);
    m_rightEncoderObj.setDistancePerPulse(0.1524 * Math.PI / 1024);

    m_leftEncoder.setPosition(0);
    m_leftEncoder.setPosition(0);
    ahrs.reset();

    m_rightLeaderMotor.setIdleMode(IdleMode.kBrake);
    m_rightFollowerMotor.setIdleMode(IdleMode.kBrake);
    m_leftLeaderMotor.setIdleMode(IdleMode.kBrake);
    m_leftFollowerMotor.setIdleMode(IdleMode.kBrake);

    m_rightLeaderMotor.setInverted(true);
    m_leftLeaderMotor.setInverted(false);

    m_leftLeaderMotor.setSmartCurrentLimit(45);
    m_leftFollowerMotor.setSmartCurrentLimit(45);
    m_rightLeaderMotor.setSmartCurrentLimit(45);
    m_rightFollowerMotor.setSmartCurrentLimit(45);
    // sim

    m_odometry = new DifferentialDriveOdometry(
        m_gyro.getRotation2d(),
        m_leftEncoderObj.getDistance(),
        m_rightEncoderObj.getDistance(),
        new Pose2d(1, 1, new Rotation2d()));

    m_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(DriveConstants.trackWidthInches));

    m_chassisSpeeds = new ChassisSpeeds(0, 0, 0);

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

    m_odometry.update(
        m_gyro.getRotation2d(),
        m_leftEncoderObj.getDistance(),
        m_rightEncoderObj.getDistance());

    // sim
    m_fieldSim.setRobotPose(getPose());

    SmartDashboard.putNumber("Yaw", ahrs.getAngle());
    SmartDashboard.putNumber("disp", m_rightEncoder.getPosition());

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

  public void driveArcade(double xForward, double zRotation) {
    m_drive.arcadeDrive(xForward, zRotation);
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

  public double getPositionSim() {
    return m_rightEncoderObj.getDistance();
  }

  public double getPosition() {
    return m_rightEncoder.getPosition();
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

  public ChassisSpeeds getChassisSpeeds() {
    return m_kinematics
        .toChassisSpeeds(new DifferentialDriveWheelSpeeds(m_leftEncoderObj.getRate(), m_rightEncoderObj.getRate()));
  }

  public double getDisplacementX() {
    return m_odometry.getPoseMeters().getX();
  }

  public double getDisplacementY() {
    return m_odometry.getPoseMeters().getY();
  }

  public double getHeading() {// -180 to 180
    return Math.IEEEremainder(ahrs.getAngle(), 360);
  }

  public double getHeadingSim() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360);
  }

  public double modAngle(double angle) {
    return Math.IEEEremainder(angle, 360);
  }
}