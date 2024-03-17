// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopCommands.drive.encoder_gyro;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToDistanceSpeed extends Command {

  private final DriveSubsystem m_driveSubsystem;
  private final double targetDistanceInMetres;
  private double initialPosition;

  private final PIDController drivePID1 = new PIDController(0.5, 0, 0);
  private final PIDController drivePID2 = new PIDController(1, 0, 0);

  /** Creates a new DriveToDistance. 
   * Drives a set distance, positive value drives backward
  */
  public DriveToDistanceSpeed(DriveSubsystem driveSubsystem, double targetInMetres) {
    
    m_driveSubsystem = driveSubsystem;
    targetDistanceInMetres = targetInMetres;
    drivePID1.setTolerance(0.05);
    drivePID2.setTolerance(0.05);
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
    double speed;
    if(Math.abs(m_driveSubsystem.getPosition() - (initialPosition + targetDistanceInMetres)) < 0.5){
      speed = drivePID2.calculate(m_driveSubsystem.getPosition(), initialPosition + targetDistanceInMetres);
    }else{
      speed = drivePID1.calculate(m_driveSubsystem.getPosition(), initialPosition + targetDistanceInMetres);
    }

    double FF;
    SmartDashboard.putNumber("Error", speed);

    if(speed > 0){
      FF = 0.01;
    }else{
      FF = -0.01;
    }
    
    m_driveSubsystem.driveSpeed((speed + FF) * 4000);

    // SmartDashboard.putNumber("speed", speed);
    // SmartDashboard.putNumber("target distance", initialPosition + targetDistanceInMetres);
    // SmartDashboard.putNumber("current position", m_driveSubsystem.getPosition());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_driveSubsystem.getPosition() - (initialPosition + targetDistanceInMetres)) < 0.05;
  }

  public double getDistanceTravelled(){
    return m_driveSubsystem.getPosition() - (initialPosition + targetDistanceInMetres);
  }
}
