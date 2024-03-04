// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private final PWMSparkMax m_intakeMotor = new PWMSparkMax(IntakeConstants.intakePWMid);
  //private final DigitalInput intakeSensor = new DigitalInput(IntakeConstants.beamBrake);
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(IntakeConstants.kSFeedForward,
      IntakeConstants.kVFeedForward, IntakeConstants.kAFeedForward); // find estimates

  public double targetPower = 0;

  public IntakeSubsystem() {
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

  @Override
  public void periodic() {
    SmartDashboard.putNumber("target power", targetPower);
    SmartDashboard.putNumber("motor power", m_intakeMotor.get());

    if (false) {//intakeSensor.get()
      // Apply feedforward
      m_intakeMotor.set(feedforward.calculate(10, 0));
    } else {
      m_intakeMotor.set(targetPower);
    }

    // This method will be called once per scheduler run
  }
}
