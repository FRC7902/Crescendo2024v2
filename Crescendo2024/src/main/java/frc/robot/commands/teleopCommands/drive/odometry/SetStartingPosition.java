// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopCommands.drive.odometry;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class SetStartingPosition extends Command {
  private DriveSubsystem m_driveSubsystem;
  double m_degrees;
  double m_x;
  double m_y;
  /** Creates a new SetStartingPosition. */
  public SetStartingPosition(DriveSubsystem driveSubsystem, double degrees, double x, double y) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSubsystem;
    m_degrees = degrees;
    m_x = x;
    m_y = y;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveSubsystem.setStartingPosition(m_degrees, m_x, m_y);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
