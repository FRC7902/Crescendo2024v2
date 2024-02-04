// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class DriveSubsystem extends SubsystemBase {

 private final CANSparkMax m_leftLeaderMotor = new CANSparkMax(DriveConstants.leftFrontCANID, CANSparkMax.MotorType.kBrushless);
 private final CANSparkMax m_leftFollowerMotor = new CANSparkMax(DriveConstants.leftBackCANID, CANSparkMax.MotorType.kBrushless);
 private final CANSparkMax m_rightLeaderMotor = new CANSparkMax(DriveConstants.rightFrontCANID, CANSparkMax.MotorType.kBrushless);
 private final CANSparkMax m_rightFollowerMotor = new CANSparkMax(DriveConstants.rightBackCANID, CANSparkMax.MotorType.kBrushless);

 private final DifferentialDrive m_drive = new DifferentialDrive(m_leftLeaderMotor::set, m_rightLeaderMotor::set);

 private final RelativeEncoder m_leftEncoder = m_leftLeaderMotor.getEncoder();
 private final RelativeEncoder m_rightEncoder = m_rightLeaderMotor.getEncoder();

 private final AnalogGyro m_gyro = new AnalogGyro(DriveConstants.GyroCAN);

  //Simulation Stuff
  private DifferentialDriveOdometry m_odometry;
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


      m_leftLeaderMotor.follow(m_leftFollowerMotor);
      m_rightLeaderMotor.follow(m_rightFollowerMotor);

     m_leftEncoder.setPositionConversionFactor(0.1524 * Math.PI / 1024);
     m_rightEncoder.setPositionConversionFactor(0.1524 * Math.PI / 1024);

      m_leftEncoder.setPosition(0);
      m_leftEncoder.setPosition(0);

      m_rightLeaderMotor.setInverted(true);
      m_leftLeaderMotor.setInverted(false);

      m_leftLeaderMotor.setSmartCurrentLimit(45);
      m_leftFollowerMotor.setSmartCurrentLimit(45);
      m_rightLeaderMotor.setSmartCurrentLimit(45);
      m_rightFollowerMotor.setSmartCurrentLimit(45);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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

}
