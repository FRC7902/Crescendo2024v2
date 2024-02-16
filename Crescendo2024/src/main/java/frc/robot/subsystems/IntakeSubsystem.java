// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
    private final WPI_TalonSRX  intakeMotor = new WPI_TalonSRX (Constants.IntakeConstants.kIntakeCANID1);
    private final WPI_TalonSRX  intakeMotor2 = new WPI_TalonSRX (Constants.IntakeConstants.kIntakeCANID2);

  public IntakeSubsystem() {
      intakeMotor.setInverted(false);
      intakeMotor2.setInverted(true);

      currentConfig();
      stopMotor();

  }
  
  public void currentConfig(){
    intakeMotor.configPeakCurrentLimit(35, 10); 
    intakeMotor.configPeakCurrentDuration(200, 10); 
    intakeMotor.configContinuousCurrentLimit(30, 10);
    intakeMotor.enableCurrentLimit(true);
  }

  public void stopMotor(){
    intakeMotor.stopMotor();
    intakeMotor2.stopMotor();
  }

  public void setPower(double power){
    intakeMotor.set(power);
    intakeMotor2.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
