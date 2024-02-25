// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopCommands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToDistance extends Command {

  private final DriveSubsystem m_driveSubsystem;
  private final double targetDistance;
  private double initialPosition;

  private final PIDController drivePID = new PIDController(1, 0, 0);

  /** Creates a new DriveToDistance. */
  public DriveToDistance(DriveSubsystem driveSubsystem, double target) {

    m_driveSubsystem = driveSubsystem;
    targetDistance = target;
    drivePID.setTolerance(0.00001, 1);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialPosition = m_driveSubsystem.getPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = drivePID.calculate(m_driveSubsystem.getPosition(), initialPosition + targetDistance);
    m_driveSubsystem.driveRaw(speed);

    SmartDashboard.putNumber("speed", speed);
    SmartDashboard.putNumber("target distance", initialPosition + targetDistance);
    SmartDashboard.putNumber("current position", m_driveSubsystem.getPosition());

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
