// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class FeedNote extends Command {
  private IntakeSubsystem m_intake;

  /** Creates a new Suck. */
  public FeedNote(IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;

    // intake a note, when the beam brake is hit, stop spinning and apply a small
    // feedforward to hold onto the note

    // seperate command: feed the note into the shooter (ignore the beam brake)
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.stopMotor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (true) {
      m_intake.setTargetPower(Constants.IntakeConstants.feedingSpeed);
    } else {
      m_intake.setTargetPower(Constants.IntakeConstants.holdPower);
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
