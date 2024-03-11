// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopCommands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToDistance extends Command {

  private final DriveSubsystem m_driveSubsystem;
  private final double targetDistanceInMetres;
  private double initialPosition;

  private final PIDController drivePID = new PIDController(0.25, 0, 0);

  /** Creates a new DriveToDistance. */
  public DriveToDistance(DriveSubsystem driveSubsystem, double targetInMetres) {
    
    m_driveSubsystem = driveSubsystem;
    targetDistanceInMetres = targetInMetres;
    drivePID.setTolerance(0.1);
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialPosition = m_driveSubsystem.getPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = drivePID.calculate(m_driveSubsystem.getPosition(), initialPosition + targetDistanceInMetres);
    double FF;
    SmartDashboard.putNumber("Error", speed);

    if(speed > 0){
      FF = 0.01;
    }else{
      FF = -0.01;
    }
    
    m_driveSubsystem.driveRaw(speed + FF);

    SmartDashboard.putNumber("speed", speed);
    SmartDashboard.putNumber("target distance", initialPosition + targetDistanceInMetres);
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
