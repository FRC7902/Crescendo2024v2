// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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
  public int targetSpeedCounter = 0;

  public SparkPIDController speedPID;
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

    speedPID = m_leaderMotor.getPIDController();
    speedPID.setReference(0, CANSparkBase.ControlType.kVelocity);
    speedPID = m_leaderMotor.getPIDController();
    speedPID.setReference(0, CANSparkBase.ControlType.kVelocity);
    speedPID.setOutputRange(0, 1);
    
    m_leaderMotor.setSmartCurrentLimit(45);
    m_followerMotor.setSmartCurrentLimit(45);

    setPID(ShooterConstants.shooterkP, ShooterConstants.shooterkI, ShooterConstants.shooterkD);
  }

  public void setSpeed(double speed) {
    speedPID.setReference(speed, ControlType.kVelocity);
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
    if (200 > Math.abs((0.75 * targetSpeed) - sparkEncoder.getVelocity())) {
      targetSpeedCounter++;
    } else {
      targetSpeedCounter = 0;
    }

    if(targetSpeedCounter > 30){
      return true;
    }else{
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
    speedPID.setP(kP);
    speedPID.setI(kI);
    speedPID.setD(kD);
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
    // SmartDashboard.putString("ShooterSubsystem/Shooter Status", status);
    // SmartDashboard.putNumber("Shooter speed", sparkEncoder.getVelocity());
    SmartDashboard.putNumber("Target Speed", targetSpeed);
    SmartDashboard.putBoolean("at target speed", atTargetSpeed());

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