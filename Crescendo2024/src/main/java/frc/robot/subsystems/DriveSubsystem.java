// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.xml.validation.SchemaFactory;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
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



public class DriveSubsystem extends SubsystemBase {

 private final CANSparkMax m_leftLeaderMotor = new CANSparkMax(DriveConstants.leftFrontCANID, CANSparkMax.MotorType.kBrushless);
 private final CANSparkMax m_leftFollowerMotor = new CANSparkMax(DriveConstants.leftBackCANID, CANSparkMax.MotorType.kBrushless);
 private final CANSparkMax m_rightLeaderMotor = new CANSparkMax(DriveConstants.rightFrontCANID, CANSparkMax.MotorType.kBrushless);
 private final CANSparkMax m_rightFollowerMotor = new CANSparkMax(DriveConstants.rightBackCANID, CANSparkMax.MotorType.kBrushless);

 private final DifferentialDrive m_drive;

 private final RelativeEncoder m_leftEncoder = m_leftLeaderMotor.getEncoder();
 private final RelativeEncoder m_rightEncoder = m_rightLeaderMotor.getEncoder();

 private static PigeonIMU m_pigeon = new PigeonIMU(30);


 private final AnalogGyro m_gyro = new AnalogGyro(0);
 
private DifferentialDriveOdometry m_odometry;

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

    // m_leftLeaderMotor.restoreFactoryDefaults(); //Safety precautions, but reccomended
    // m_leftFollowerMotor.restoreFactoryDefaults();
    // m_rightLeaderMotor.restoreFactoryDefaults();
    // m_rightFollowerMotor.restoreFactoryDefaults();
    
    m_leftFollowerMotor.follow(m_leftLeaderMotor);
    m_rightFollowerMotor.follow(m_rightLeaderMotor);
    
    m_drive = new DifferentialDrive(m_leftLeaderMotor, m_rightLeaderMotor);
    
     m_leftEncoder.setPositionConversionFactor(0.1524 * Math.PI / 1024);
     m_rightEncoder.setPositionConversionFactor(0.1524 * Math.PI / 1024);

      m_leftEncoder.setPosition(0);
      m_leftEncoder.setPosition(0);

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


      //sim 
      m_driveTrainSim = DifferentialDrivetrainSim.createKitbotSim( //CHANGE AS NEEDED!!
        KitbotMotor.kDualCIMPerSide, 
        KitbotGearing.k10p71, 
        KitbotWheelSize.kSixInch, 
        null
        );

      m_fieldSim = new Field2d();
      SmartDashboard.putData("Field", m_fieldSim);

    //    m_leftEncoderSim = new EncoderSim(m_leftEncoderObj);
    // m_rightEncoderSim = new EncoderSim(m_rightEncoderObj);
       m_gyroSim = new AnalogGyroSim(m_gyro);

  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Yaw", m_pigeon.getYaw());
    // This method will be called once per scheduler run

    //sim
    // m_fieldSim.setRobotPose(getPose());
  }

  @Override 
  public void simulationPeriodic() {

    // This method will be called once per scheduler run during simulation
    //connect the motors to update the drivetrain
    
    m_driveTrainSim.setInputs(
      m_leftLeaderMotor.get() * RobotController.getBatteryVoltage(),
      m_rightLeaderMotor.get() * RobotController.getBatteryVoltage()
    );
    
    m_driveTrainSim.update(0.02);

    m_leftEncoderSim.setDistance(m_driveTrainSim.getLeftPositionMeters());
    m_rightEncoderSim.setDistance(m_driveTrainSim.getRightPositionMeters());

    m_leftEncoderSim.setRate(m_driveTrainSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setRate(m_driveTrainSim.getRightVelocityMetersPerSecond());

    m_gyroSim.setAngle(-m_driveTrainSim.getHeading().getDegrees());

    SmartDashboard.putNumber("angle", getHeading());
    // SmartDashboard.putNumber("angle2", getHeadingCase2());
    SmartDashboard.putNumber("DisplacementX", getDisplacementX());

  } 

  public void driveArcade(double xForward, double zRotation){
    m_drive.arcadeDrive(xForward, zRotation);
  }


  public void driveRaw(double power){
    m_leftLeaderMotor.set(power);
    m_rightLeaderMotor.set(power);
  }

  public void turn(double amount){
    m_leftLeaderMotor.set(amount);
    m_rightLeaderMotor.set(-amount);
  }
  public double getPosition(){
    return m_rightEncoder.getPosition();
  }

  public void resetEncoders(){
    m_rightEncoder.setPosition(0);
    m_leftEncoder.setPosition(0);
  }

  public void stopMotors(){
    m_leftLeaderMotor.stopMotor();
    m_rightLeaderMotor.stopMotor();
  }

  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }

  public double getDisplacementX(){
    return m_odometry.getPoseMeters().getX();
  }

  public double getDisplacementY(){
    return m_odometry.getPoseMeters().getY();
  }

  public double getHeading(){//-180 to 180
    return Math.IEEEremainder(m_pigeon.getYaw(), 360);
  }

  public double modAngle(double angle){
    return Math.IEEEremainder(angle, 360);
  }


}
