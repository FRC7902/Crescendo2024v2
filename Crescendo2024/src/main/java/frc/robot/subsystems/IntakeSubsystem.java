// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private final PWMSparkMax m_intakeMotor = new PWMSparkMax(IntakeConstants.intakePWMid);
  private final DigitalInput intakeSensor = new DigitalInput(IntakeConstants.beamBrakePort);
  // private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(IntakeConstants.kSFeedForward,
  //     IntakeConstants.kVFeedForward, IntakeConstants.kAFeedForward); // find estimates
  private final XboxController m_operatorStick;
    
  public double targetPower = 0;
  public boolean isShooting = false;

  public IntakeSubsystem(XboxController operatorStick) {
    m_operatorStick = operatorStick;
    stopMotor();
    m_intakeMotor.setInverted(true);
    // m_intakeMotor.(IntakeConstants.intakeCurrentLimit); CURRENT LIMIT

  }

  public void stopMotor() {
    m_intakeMotor.stopMotor();
  }

  public void setPower(double power) {
    m_intakeMotor.set(power);
  }

  // public boolean getSensor() {
  //   return intakeSensor.get();
  // }

  public void setTargetPower(double target) {
    targetPower = target;
  }

  public void setShootingStatus(boolean status){
    isShooting = status;
  }

  public boolean getBeamBrake(){
    return intakeSensor.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("target power", targetPower);
    SmartDashboard.putNumber("motor power", m_intakeMotor.get());
    SmartDashboard.putBoolean("Beam brake", intakeSensor.get());

    if (!intakeSensor.get() && !isShooting) {
      m_intakeMotor.set(-0.3);
      m_operatorStick.setRumble(RumbleType.kBothRumble, 1);
    } else {
      m_intakeMotor.set(targetPower);
      m_operatorStick.setRumble(RumbleType.kBothRumble, 0);
    }

    // This method will be called once per scheduler run
  }
}
