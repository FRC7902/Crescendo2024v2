// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopCommands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToDistance extends Command {

  private final DriveSubsystem m_driveSubsystem;
  private final double targetDistance;

  private final PIDController drivePID = new PIDController(0, 0, 0);

  /** Creates a new DriveToDistance. */
  public DriveToDistance(DriveSubsystem driveSubsystem, double target) {

    m_driveSubsystem = driveSubsystem;
    targetDistance = target;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      double speed = drivePID.calculate(targetDistance);
      m_driveSubsystem.driveRaw(speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivePID.atSetpoint();
  }
}
