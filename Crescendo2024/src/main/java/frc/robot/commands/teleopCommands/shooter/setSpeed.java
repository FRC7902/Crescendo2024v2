// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopCommands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class setSpeed extends Command {
  private ShooterSubsystem m_shooter;
  private double targetSpeed;

  /** Creates a new setSpeed. */
  public setSpeed(ShooterSubsystem shooter, double speedRPM) {
    m_shooter = shooter;
    targetSpeed = speedRPM;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setTargetSpeed(targetSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
