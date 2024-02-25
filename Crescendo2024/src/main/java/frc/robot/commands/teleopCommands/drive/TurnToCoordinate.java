// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopCommands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FireBirdsUtils;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToCoordinate extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final double targetX;
  private final double targetY;
  private static double trueTarget;
  private final PIDController turnPID = new PIDController(0.05, 0, 0);
  private final FireBirdsUtils util = new FireBirdsUtils();
  /** Creates a new TurnToCoordinate. */
  public TurnToCoordinate(DriveSubsystem drive, double x, double y) {
    m_driveSubsystem = drive;
    targetX = x;
    targetY = y;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double dx = targetX - m_driveSubsystem.getDisplacementX();
    double dy = targetY - m_driveSubsystem.getDisplacementY();
    turnPID.setTolerance(1, 1);

    double relatedAngle = util.radsToDegrees(Math.atan(dx/dy));

    if(dx < 0 && dy < 0){
      trueTarget = 90 + relatedAngle;
    }else if(dx < 0 && dy > 0){
      trueTarget = relatedAngle - 90;
    }else if(dx > 0 && dy > 0){
      trueTarget = -90 + relatedAngle;
    }else if (dx > 0 && dy < 0){
      trueTarget = relatedAngle + 90;
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed;
    speed = turnPID.calculate(convertRange(m_driveSubsystem.getHeading()), trueTarget);

    SmartDashboard.putNumber("target angle", trueTarget);
    SmartDashboard.putNumber("Current angle", convertRange(m_driveSubsystem.getHeading()));
    m_driveSubsystem.turn(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turnPID.atSetpoint();
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
}
