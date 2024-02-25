// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.teleopCommands;

import frc.robot.FireBirdsUtils;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbDistance extends Command {
    private ClimbSubsystem m_climbSubsystem;
    private double initialDistance = 0;
    private double fullyExtendedDistance = 200;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ClimbDistance(ClimbSubsystem climbSubsystem) {
    m_climbSubsystem = climbSubsystem;
    initialDistance = m_climbSubsystem.encoderDistance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climbSubsystem.setNewTargetPosition(util.degToCTRESensorUnits(fullyExtendedDistance, ArmConstants.EncoderCPR));
    SmartDashboard.putNumber("targetAngle", (initialDistance + fullyExtendedDistance));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climbSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
