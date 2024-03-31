// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WinchSubsystem extends SubsystemBase {
  private final CANSparkMax m_leaderMotor = new CANSparkMax(40, CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_followerMotor = new CANSparkMax(41, CANSparkMax.MotorType.kBrushless);

  /** Creates a new WinchSubsystem. */
  public WinchSubsystem() {
    m_followerMotor.follow(m_leaderMotor);
    m_leaderMotor.setSmartCurrentLimit(45);
    m_followerMotor.setSmartCurrentLimit(45);
  }

  public void setPower(double power){
    m_leaderMotor.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
