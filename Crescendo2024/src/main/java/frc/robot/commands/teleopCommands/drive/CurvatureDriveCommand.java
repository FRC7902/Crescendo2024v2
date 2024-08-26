// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopCommands.drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class CurvatureDriveCommand extends Command {

  private final DriveSubsystem m_driveSubsystem;
  private final XboxController m_driverController;

  /** Creates a new CurvatureDriveCommand. */
  public CurvatureDriveCommand(DriveSubsystem driveSubsystem, XboxController driverController) {
    m_driveSubsystem = driveSubsystem;
    m_driverController = driverController;
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.driveCurvature(
        m_driverController.getRawAxis(Constants.IOConstants.kLY),
        m_driverController.getRawAxis(Constants.IOConstants.kRX));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
