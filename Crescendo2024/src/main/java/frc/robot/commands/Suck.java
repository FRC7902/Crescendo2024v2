// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class Suck extends Command {
  private IntakeSubsystem m_intake;
  private boolean m_beamBrakeHit;
  /** Creates a new Suck. */
  public Suck(IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    m_beamBrakeHit = false;
    addRequirements(intake);


    //intake a note, when the beam brake is hit, stop spinning and apply a small feedforward to hold onto the note

    //seperate command: feed the note into the shooter (ignore the beam brake)
  }
  
  public void setBeamBrakeHit(boolean beamBrakeHit) {
    m_beamBrakeHit = beamBrakeHit;

  }
    
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.stopMotor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_beamBrakeHit) {
      m_intake.setPower(Constants.IntakeConstants.kIntakeHoldPower);
      } else {
      m_intake.setPower(Constants.IntakeConstants.kIntakeSuckingSpeed);
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_intake.stopMotor();
    
    }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
