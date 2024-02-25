// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopCommands.drive;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToTag extends Command {

  private final DriveSubsystem m_driveSubsystem;
  private final double targetAngle;
  private double trueTarget;
  private boolean isAdditive;
  private double initialAngle;
  private final PIDController turnPID = new PIDController(0.05, 0, 0);

  /** Creates a new TurnToAngle. */
  public TurnToTag(DriveSubsystem drive, PhotonCamera m_camera) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = drive;

    var res = m_camera.getLatestResult();

    if(res.hasTargets()){
      targetAngle = res.getBestTarget().getYaw();
    }else{
      targetAngle = 0;
    }
    turnPID.setTolerance(1, 1);
    initialAngle = m_driveSubsystem.getHeading();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialAngle = m_driveSubsystem.getHeading();
    trueTarget = Math.round(m_driveSubsystem.modAngle(initialAngle - targetAngle));
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed;
    speed = turnPID.calculate(convertRange(m_driveSubsystem.getHeading()), trueTarget);

    SmartDashboard.putNumber("target angle", trueTarget);
    SmartDashboard.putNumber("Current angle", convertRange(m_driveSubsystem.getHeading()));
    m_driveSubsystem.turn(speed * 0.1);
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
    return turnPID.atSetpoint();
  }
}
