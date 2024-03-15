// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopCommands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToAngleOdometry extends Command {

  private final DriveSubsystem m_driveSubsystem;
  private final double targetAngle;
  private double trueTarget;
  private boolean isAdditive;
  private double initialAngle;
  private final PIDController turnPID1 = new PIDController(0.0025, 0.0025, 0);
  private final PIDController turnPID2 = new PIDController(0.005, 0, 0);
  private final PIDController turnPID3 = new PIDController(0.002, 0, 0);


  /** Creates a new TurnToAngle. */
  public TurnToAngleOdometry(DriveSubsystem drive, double angle, boolean IsAdditive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = drive;
    targetAngle = angle;
    isAdditive = IsAdditive;
    turnPID1.setTolerance(1.5);
    turnPID2.setTolerance(1.5);
    turnPID3.setTolerance(1.5);
    initialAngle = m_driveSubsystem.getEstimatedRotation();
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialAngle = m_driveSubsystem.getEstimatedRotation();
    if (isAdditive) {
      trueTarget = Math.round(m_driveSubsystem.modAngle(targetAngle + initialAngle));
    } else {
      trueTarget = targetAngle;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed;
    
    if(Math.abs(convertRange(m_driveSubsystem.getEstimatedRotation()) - trueTarget) < 5){
      speed = turnPID3.calculate(convertRange(m_driveSubsystem.getEstimatedRotation()), trueTarget);
    }else if(Math.abs(convertRange(m_driveSubsystem.getEstimatedRotation()) - trueTarget) < 20){
      speed = turnPID2.calculate(convertRange(m_driveSubsystem.getEstimatedRotation()), trueTarget);
    }else{
      speed = turnPID1.calculate(convertRange(m_driveSubsystem.getEstimatedRotation()), trueTarget);
    }

    SmartDashboard.putNumber("turning error", Math.abs(convertRange(m_driveSubsystem.getEstimatedRotation()) - trueTarget));
    SmartDashboard.putBoolean("atSetpoint", turnPID1.atSetpoint() || turnPID2.atSetpoint() || turnPID3.atSetpoint());
    double FF;

    if(convertRange(m_driveSubsystem.getEstimatedRotation()) - trueTarget > 0){
      FF = -0.12;
    }else if(convertRange(m_driveSubsystem.getEstimatedRotation()) - trueTarget < 0){
      FF = 0.12;
    }else{
      FF = 0;
    }

    m_driveSubsystem.turn(speed + FF);
  }

  public double convertRange(double angle) {// range from targetAngle +- 180
    if (angle > trueTarget + 180) {
      return angle - 360;
    } else if (angle < trueTarget - 180) {
      return angle + 360;
    } else {
      return angle;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turnPID1.atSetpoint() || turnPID2.atSetpoint() || turnPID3.atSetpoint();
    // return Math.abs(convertRange(m_driveSubsystem.getEstimatedRotation()) - trueTarget) < 1;
  }

}
