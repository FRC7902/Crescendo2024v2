// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private final WPI_TalonSRX intakeMotor = new WPI_TalonSRX(IntakeConstants.intakeCANID1);
  private final DigitalInput intakeSensor = new DigitalInput(IntakeConstants.beamBrake);
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA); //find estimates 

  //simulation??? 

  public IntakeSubsystem() {
      stopMotor();
  }

  public void stopMotor(){
    intakeMotor.stopMotor();
  }

  public void setPower(double power){
    intakeMotor.set(power);
  }

  public boolean getSensor() {
    return intakeSensor.get();
  }

  @Override
  public void periodic() {
 
    if (intakeSensor.get()) {
            // Apply feedforward
            intakeMotor.set(feedforward.calculate(10, 0));
        } else {
            // Do other periodic tasks if needed
     }

    // This method will be called once per scheduler run
  }
}
