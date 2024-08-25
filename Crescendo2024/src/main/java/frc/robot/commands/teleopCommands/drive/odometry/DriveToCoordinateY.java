// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopCommands.drive.odometry;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToCoordinateY extends Command {

  private final DriveSubsystem m_driveSubsystem;
  private final double targetCoordinateY;
  private final PIDController drivePID = new PIDController(0.25, 0, 0);

  /**
   * Creates a new DriveToDistance.
   * Drives a set distance, positive value drives backward
   */
  public DriveToCoordinateY(DriveSubsystem driveSubsystem, double targetY) {

    m_driveSubsystem = driveSubsystem;
    targetCoordinateY = targetY;
    drivePID.setTolerance(0.1);
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = drivePID.calculate(m_driveSubsystem.getDisplacementY(), targetCoordinateY);
    double FF;

    if (speed > 0) {
      FF = 0.01;
    } else {
      FF = -0.01;
    }

    m_driveSubsystem.driveRaw(speed + FF);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivePID.atSetpoint();
  }
}
