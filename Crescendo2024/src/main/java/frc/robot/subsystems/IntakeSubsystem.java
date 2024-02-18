// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private final PWMSparkMax intakeMotor = new PWMSparkMax(IntakeConstants.kIntakeCANID1);
  public IntakeSubsystem() {
      stopMotor();
  }

  public void stopMotor(){
    intakeMotor.stopMotor();
  }

  public void setPower(double power){
    intakeMotor.set(power);
  }

  @Override
  public void periodic() {
    boolean beamBrakeHit = true;
    //if beam brake is hit, apply feedforward
    if (beamBrakeHit) {
            // Apply feedforward
            intakeMotor.set(IntakeConstants.kFeedforwardPower);
        } else {
            // Do other periodic tasks if needed
     }

    // This method will be called once per scheduler run
  }
}
