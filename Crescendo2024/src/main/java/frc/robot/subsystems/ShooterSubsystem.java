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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

  public final CANSparkMax master = new CANSparkMax(ShooterConstants.kMasterCAN,
      CANSparkMax.MotorType.kBrushless);// left
  public final CANSparkMax follower = new CANSparkMax(ShooterConstants.kFollowerCAN,
      CANSparkMax.MotorType.kBrushless); // right

  public double targetSpeed = 0;

  public SparkPIDController masterspeedPID;
  public final RelativeEncoder sparkEncoder = master.getEncoder();

  public String status = "Off";

  public ShooterSubsystem() {

    follower.follow(master);

    master.setInverted(false);
    follower.setInverted(true);

    master.setOpenLoopRampRate(ShooterConstants.kRampTime);
    follower.setOpenLoopRampRate(ShooterConstants.kRampTime);

    master.setIdleMode(IdleMode.kBrake);
    follower.setIdleMode(IdleMode.kBrake);

    masterspeedPID = master.getPIDController();
    masterspeedPID.setReference(0, CANSparkBase.ControlType.kVelocity);

    master.setSmartCurrentLimit(45);
    follower.setSmartCurrentLimit(45);

    // sparkEncoder.setVelocityConversionFactor();
    setPID(ShooterConstants.shooterkP, ShooterConstants.shooterkI, ShooterConstants.shooterkD);
  }

  public void setSpeed(double speed) {
    masterspeedPID.setReference(speed, ControlType.kVelocity);
    if (speed > 0) {
      status = "Shooting...";
    } else if (speed < 0) {
      status = "Reversing...";
    }
  }

  public void stop() {
    master.stopMotor();
    master.set(0.00); // numbers need to be changed
    status = "Off";
  }

  public void coast() {
    master.setIdleMode(IdleMode.kCoast);
    follower.setIdleMode(IdleMode.kCoast);
  }

  public void brake() {
    master.setIdleMode(IdleMode.kBrake);

    follower.setIdleMode(IdleMode.kBrake);
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
    masterspeedPID.setP(kP);
    masterspeedPID.setI(kI);
    masterspeedPID.setD(kD);
  }

  @Override
  public void periodic() {
    setSpeed(targetSpeed);
    SmartDashboard.putNumber("ShooterSubsystem/Shooter Power", master.getAppliedOutput());
    SmartDashboard.putNumber("ShooterSubsystem/Shooter Power 2", follower.getAppliedOutput());
    SmartDashboard.putString("ShooterSubsystem/Shooter Status", status);

    // SmartDashboard.putNumber("CompetitionView/Shooter Power",
    // master.getAppliedOutput());
    // SmartDashboard.putString("CompetitionView/Shooter Status", status);

    SmartDashboard.putNumber("ShooterSubsystem/Encoder Speed", sparkEncoder.getVelocity());
    SmartDashboard.putNumber("Target Speed", targetSpeed);

  }

  // @Override
  // public void simulationPeriodic(){
  // setTargetSpeed(targetSpeed);
  // SmartDashboard.putNumber("ShooterSubsystem/Shooter Power",
  // master.getAppliedOutput());
  // SmartDashboard.putNumber("ShooterSubsystem/Shooter Power 2",
  // follower.getAppliedOutput());
  // SmartDashboard.putString("ShooterSubsystem/Shooter Status", status);

  // SmartDashboard.putNumber("CompetitionView/Shooter Power",
  // master.getAppliedOutput());
  // SmartDashboard.putString("CompetitionView/Shooter Status", status);

  // SmartDashboard.putNumber("ShooterSubsystem/Encoder Speed",
  // sparkEncoder.getVelocity());

  // }
}