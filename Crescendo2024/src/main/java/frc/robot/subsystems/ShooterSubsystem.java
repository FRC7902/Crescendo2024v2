// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.InvertType;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.revrobotics.CANSparkBase.IdleMode;
//import com.revrobotics.CANSparkBase.IdleMode;
//import com.revrobotics.CANSparkBase;
//import com.revrobotics.CANSparkBase.IdleMode;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController; 
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import frc.robot.Constants.ShooterConstants;


public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

  public final CANSparkMax master = new CANSparkMax(Constants.ShooterConstants.kMasterCAN,
      CANSparkMax.MotorType.kBrushless);// left
  public final CANSparkMax follower = new CANSparkMax(Constants.ShooterConstants.kFollowerCAN,
      CANSparkMax.MotorType.kBrushless); // right

  // Encoder
  public final Encoder encoder = new Encoder(5, 4);
  public final PIDController controller = new PIDController(Constants.ShooterConstants.kTolerance, 0, 0);

  public final SparkPIDController speedPID;

  public String status = "Off";

  public ShooterSubsystem() {

    follower.follow(master);

    master.setInverted(false);
    follower.setInverted(true);

    master.setOpenLoopRampRate(Constants.ShooterConstants.kRampTime);
    follower.setOpenLoopRampRate(Constants.ShooterConstants.kRampTime);

    master.setIdleMode(IdleMode.kBrake);
    follower.setIdleMode(IdleMode.kBrake);

    speedPID = master.getPIDController();

  }

  public void setSpeed(double speed) {
    master.set(speed);
    if (speed > 0) {
      status = "Shooting...";
    } else if (speed < 0) {
      status = "Reversing...";
    }
  }

  public void stop() {
    master.stopMotor();
    master.set(-0.00); // numbers need to be changed
    status = "Off";
  }

  public void coast() {
    master.setIdleMode(IdleMode.kCoast);
    follower.setIdleMode(IdleMode.kCoast);
  }

  public void brake() {
    master.setIdleMode(IdleMode.kBrake);;
    follower.setIdleMode(IdleMode.kBrake);
  }

  public boolean atTargetSpeed() {
    return controller.atSetpoint();
  }

  public void PIDSpeed(int setpoint) {
    master.set(controller.calculate(encoder.getRate(), setpoint));
  }

  public void amp() {
    master.set(Constants.ShooterConstants.kAmpSpeed);
    follower.set(Constants.ShooterConstants.kAmpSpeed);
  }

  public void speaker() {
    master.set(Constants.ShooterConstants.kSpeakerSpeed); 
    follower.set(Constants.ShooterConstants.kSpeakerSpeed); 
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("ShooterSubsystem/Shooter Power", master.getAppliedOutput());
    SmartDashboard.putNumber("ShooterSubsystem/Shooter Power 2", follower.getAppliedOutput());
    SmartDashboard.putString("ShooterSubsystem/Shooter Status", status);

    SmartDashboard.putNumber("CompetitionView/Shooter Power", master.getAppliedOutput());
    SmartDashboard.putString("CompetitionView/Shooter Status", status);

    SmartDashboard.putNumber("ShooterSubsystem/Encoder Speed", encoder.getRate());

  }
}