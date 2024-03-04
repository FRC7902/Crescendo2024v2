// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
//import com.ctre.phoenix.motorcontrol.InvertType;
//import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.revrobotics.CANSparkBase.IdleMode;
//import com.revrobotics.CANSparkBase.IdleMode;
//import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

  public final CANSparkMax m_leaderMotor = new CANSparkMax(ShooterConstants.kLeaderCAN,
      CANSparkMax.MotorType.kBrushless);// left
  public final CANSparkMax m_followerMotor = new CANSparkMax(ShooterConstants.kFollowerCAN,
      CANSparkMax.MotorType.kBrushless); // right

  public double targetSpeed = 0;

  public SparkPIDController m_leaderMotorspeedPID;
  public final RelativeEncoder sparkEncoder = m_leaderMotor.getEncoder();

  public String status = "Off";

  public ShooterSubsystem() {

    m_followerMotor.follow(m_leaderMotor);

    m_leaderMotor.setInverted(false);
    m_followerMotor.setInverted(true);

    m_leaderMotor.setOpenLoopRampRate(ShooterConstants.kRampTime);
    m_followerMotor.setOpenLoopRampRate(ShooterConstants.kRampTime);

    m_leaderMotor.setIdleMode(IdleMode.kBrake);
    m_followerMotor.setIdleMode(IdleMode.kBrake);

    m_leaderMotorspeedPID = m_leaderMotor.getPIDController();
    m_leaderMotorspeedPID.setReference(0, CANSparkBase.ControlType.kVelocity);

    m_leaderMotor.setSmartCurrentLimit(45);
    m_followerMotor.setSmartCurrentLimit(45);

    // sparkEncoder.setVelocityConversionFactor();
    setPID(ShooterConstants.shooterkP, ShooterConstants.shooterkI, ShooterConstants.shooterkD);
  }

  public void setSpeed(double speed) {
    m_leaderMotorspeedPID.setReference(speed, ControlType.kVelocity);
    if (speed > 0) {
      status = "Shooting...";
    } else if (speed < 0) {
      status = "Reversing...";
    }
  }

  public void stop() {
    m_leaderMotor.stopMotor();
    m_leaderMotor.set(0.00); // numbers need to be changed
    status = "Off";
  }

  public void coast() {
    m_leaderMotor.setIdleMode(IdleMode.kCoast);
    m_followerMotor.setIdleMode(IdleMode.kCoast);
  }

  public void brake() {
    m_leaderMotor.setIdleMode(IdleMode.kBrake);

    m_followerMotor.setIdleMode(IdleMode.kBrake);
  }

  public boolean atTargetSpeed() {
    if (1 > Math.abs(targetSpeed - sparkEncoder.getVelocity())) {
      return true;
    } else {
      return false;
    }
  }

  public void setTargetSpeed(double target) {
    targetSpeed = target;
  }

  public double getTargetSpeed() {
    return targetSpeed;
  }

  public void amp() {
    setTargetSpeed(ShooterConstants.kAmpSpeed);

  }

  public void speaker() {
    setTargetSpeed(ShooterConstants.kSpeakerSpeed);

  }

  public void setPID(double kP, double kI, double kD) {
    m_leaderMotorspeedPID.setP(kP);
    m_leaderMotorspeedPID.setI(kI);
    m_leaderMotorspeedPID.setD(kD);
  }

  @Override
  public void periodic() {
    if(targetSpeed == 0){
      stop();
    }else{
      setSpeed(targetSpeed);
    }
    SmartDashboard.putNumber("ShooterSubsystem/Shooter Power", m_leaderMotor.getAppliedOutput());
    SmartDashboard.putNumber("ShooterSubsystem/Shooter Power 2", m_followerMotor.getAppliedOutput());
    SmartDashboard.putString("ShooterSubsystem/Shooter Status", status);

    // SmartDashboard.putNumber("CompetitionView/Shooter Power",
    // m_leaderMotor.getAppliedOutput());
    // SmartDashboard.putString("CompetitionView/Shooter Status", status);

    SmartDashboard.putNumber("ShooterSubsystem/Encoder Speed", sparkEncoder.getVelocity());
    SmartDashboard.putNumber("Target Speed", targetSpeed);

  }

  // @Override
  // public void simulationPeriodic(){
  // setTargetSpeed(targetSpeed);
  // SmartDashboard.putNumber("ShooterSubsystem/Shooter Power",
  // m_leaderMotor.getAppliedOutput());
  // SmartDashboard.putNumber("ShooterSubsystem/Shooter Power 2",
  // m_followerMotor.getAppliedOutput());
  // SmartDashboard.putString("ShooterSubsystem/Shooter Status", status);

  // SmartDashboard.putNumber("CompetitionView/Shooter Power",
  // m_leaderMotor.getAppliedOutput());
  // SmartDashboard.putString("CompetitionView/Shooter Status", status);

  // SmartDashboard.putNumber("ShooterSubsystem/Encoder Speed",
  // sparkEncoder.getVelocity());

  // }
}